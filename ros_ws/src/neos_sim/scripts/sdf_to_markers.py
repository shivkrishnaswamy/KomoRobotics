#!/usr/bin/env python3
"""
sdf_to_markers: reads a Gazebo SDF world file and publishes every <visual> in it as a
visualization_msgs/MarkerArray, so the world can be drawn in Foxglove Studio's 3D panel
(or RViz) next to the robot, lidar scan, point cloud, etc.

Supported:
  - geometry: box, cylinder, sphere, plane, capsule, ellipsoid, mesh
  - nested models, <include> of other models (file://, model://, Fuel URLs from the local cache)
  - poses in "x y z roll pitch yaw" form (plus degrees="true" and quat_xyzw), and
    relative_to pointing at sibling links / frames / models
  - <material> diffuse/ambient colour and <transparency>

Parameters:
  world_file        (string)   SDF file to load. Default: <neos_sim share>/worlds/hallway.sdf
  frame_id          (string)   Frame the markers are expressed in. Gazebo's world origin is where
                               the robot spawned, so 'odom' lines up with the robot. Default: odom
  topic             (string)   Output topic. Default: /world_markers
  use_collision     (bool)     Draw <collision> geometry instead of <visual>. Default: false
  exclude_models    (string[]) Model names to skip, e.g. ['ground_plane']. Default: []
  publish_rate      (double)   Re-publish rate in Hz (0 = publish once, latched). Default: 1.0
  default_color     (double[]) RGBA used when a visual has no material. Default: [0.7, 0.7, 0.7, 1.0]
  resource_paths    (string[]) Extra folders to search for model:// URIs. Default: []
"""

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


# ----------------------------------------------------------------------------------------------
# Pose math (pure python so there are no extra dependencies)
# A pose is (position (x, y, z), quaternion (x, y, z, w))
# ----------------------------------------------------------------------------------------------
Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose = Tuple[Vec3, Quat]

IDENTITY: Pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    # v' = q * v * q^-1
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    r = quat_mul(quat_mul(q, qv), qc)
    return (r[0], r[1], r[2])


def compose(parent: Pose, child: Pose) -> Pose:
    """Pose of `child` (expressed in parent's frame) expressed in the frame `parent` is expressed in."""
    (pp, pq), (cp, cq) = parent, child
    rp = quat_rotate(pq, cp)
    return ((pp[0] + rp[0], pp[1] + rp[1], pp[2] + rp[2]), quat_mul(pq, cq))


def quat_from_z_to(n: Vec3) -> Quat:
    """Rotation that takes the +z axis onto vector n (used to orient <plane> by its <normal>)."""
    length = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
    nx, ny, nz = n[0] / length, n[1] / length, n[2] / length
    if nz < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)  # 180 deg about x
    # axis = z x n, angle = acos(z . n)  ->  quat = (axis, 1 + dot), normalised
    x, y, z, w = -ny, nx, 0.0, 1.0 + nz
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


# ----------------------------------------------------------------------------------------------
# Small XML helpers
# ----------------------------------------------------------------------------------------------
def floats(text: Optional[str], default: List[float]) -> List[float]:
    if text is None or not text.strip():
        return list(default)
    return [float(v) for v in text.split()]


def child_text(elem: ET.Element, tag: str) -> Optional[str]:
    c = elem.find(tag)
    return c.text if c is not None else None


def parse_pose_elem(pose_elem: Optional[ET.Element]) -> Pose:
    if pose_elem is None or not (pose_elem.text or '').strip():
        return IDENTITY
    vals = floats(pose_elem.text, [0.0] * 6)
    fmt = pose_elem.get('rotation_format', 'euler_rpy')
    pos = (vals[0], vals[1], vals[2])
    if fmt == 'quat_xyzw' and len(vals) >= 7:
        return pos, (vals[3], vals[4], vals[5], vals[6])
    rpy = vals[3:6] if len(vals) >= 6 else [0.0, 0.0, 0.0]
    if pose_elem.get('degrees', 'false').lower() in ('true', '1'):
        rpy = [math.radians(a) for a in rpy]
    return pos, quat_from_rpy(*rpy)


# ----------------------------------------------------------------------------------------------
# SDF -> list of shapes
# ----------------------------------------------------------------------------------------------
@dataclass
class Shape:
    ns: str                      # marker namespace (model/link/visual path)
    type: int                    # Marker.CUBE / CYLINDER / SPHERE / MESH_RESOURCE
    pose: Pose                   # in world frame
    scale: Vec3
    color: Optional[Tuple[float, float, float, float]]  # None -> use default / embedded materials
    mesh_uri: str = ''


@dataclass
class Scope:
    """Frames known inside one model, so poses with relative_to='<name>' can be resolved."""
    model_pose: Pose
    frames: Dict[str, Pose] = field(default_factory=dict)

    def resolve(self, pose_elem: Optional[ET.Element], default_parent: Pose) -> Pose:
        local = parse_pose_elem(pose_elem)
        rel = pose_elem.get('relative_to') if pose_elem is not None else None
        if rel and rel != '__model__' and rel in self.frames:
            return compose(self.frames[rel], local)
        if rel == '__model__':
            return compose(self.model_pose, local)
        return compose(default_parent, local)


class SdfWorldParser:
    def __init__(self, world_file: str, use_collision: bool, exclude_models: List[str],
                 resource_paths: List[str], logger):
        self.world_file = os.path.abspath(world_file)
        self.use_collision = use_collision
        self.exclude = set(exclude_models)
        self.log = logger
        self.search_paths = self._build_search_paths(resource_paths)

    # ---- resource lookup -----------------------------------------------------------------
    def _build_search_paths(self, extra: List[str]) -> List[str]:
        paths = list(extra)
        for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH', 'SDF_PATH'):
            paths += [p for p in os.environ.get(var, '').split(os.pathsep) if p]
        paths.append(os.path.dirname(self.world_file))
        paths.append(os.path.join(os.path.dirname(self.world_file), 'models'))
        return paths

    @staticmethod
    def _fuel_roots() -> List[str]:
        home = os.path.expanduser('~')
        roots = [os.path.join(home, '.gz', 'fuel'), os.path.join(home, '.ignition', 'fuel')]
        for var in ('GZ_FUEL_CACHE_PATH', 'IGN_FUEL_CACHE_PATH'):
            if os.environ.get(var):
                roots.insert(0, os.environ[var])
        return roots

    @staticmethod
    def _latest_version_dir(model_dir: str) -> str:
        """Fuel caches models as <name>/<version>/; pick the highest version if one isn't given."""
        if os.path.isfile(os.path.join(model_dir, 'model.config')):
            return model_dir
        try:
            versions = [d for d in os.listdir(model_dir) if d.isdigit()]
        except OSError:
            return model_dir
        return os.path.join(model_dir, max(versions, key=int)) if versions else model_dir

    def resolve_uri(self, uri: str, base_dir: str) -> Optional[str]:
        """Turn an SDF URI into an absolute local path, or None if it can't be found."""
        uri = uri.strip()
        if uri.startswith('file://'):
            path = uri[len('file://'):]
            return path if os.path.exists(path) else None

        if uri.startswith('model://'):
            rest = uri[len('model://'):]
            for root in self.search_paths:
                candidate = os.path.join(root, rest)
                if os.path.exists(candidate):
                    return candidate
            return None

        if uri.startswith(('http://', 'https://')):
            # e.g. https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone[/3][/files/meshes/x.dae]
            no_scheme = uri.split('://', 1)[1]
            parts = [p for p in no_scheme.split('/') if p]
            host = parts[0]
            parts = parts[1:]
            if parts and parts[0] == '1.0':
                parts = parts[1:]
            if len(parts) < 3 or parts[1] != 'models':
                return None
            owner, name = parts[0].lower(), parts[2].lower()
            tail = parts[3:]
            version = tail.pop(0) if tail and tail[0].isdigit() else None
            if tail and tail[0] == 'files':
                tail = tail[1:]
            for root in self._fuel_roots():
                model_dir = os.path.join(root, host, owner, 'models', name)
                model_dir = os.path.join(model_dir, version) if version else self._latest_version_dir(model_dir)
                candidate = os.path.join(model_dir, *tail)
                if os.path.exists(candidate):
                    return candidate
            return None

        # Plain relative or absolute path
        path = uri if os.path.isabs(uri) else os.path.join(base_dir, uri)
        return path if os.path.exists(path) else None

    @staticmethod
    def model_sdf_from_dir(model_dir: str) -> Optional[str]:
        """Find the .sdf file of a model folder using model.config (falls back to model.sdf)."""
        cfg = os.path.join(model_dir, 'model.config')
        if os.path.isfile(cfg):
            try:
                sdfs = ET.parse(cfg).getroot().findall('sdf')
                if sdfs:
                    best = max(sdfs, key=lambda e: [int(x) for x in e.get('version', '0').split('.') if x.isdigit()])
                    path = os.path.join(model_dir, (best.text or '').strip())
                    if os.path.isfile(path):
                        return path
            except ET.ParseError:
                pass
        path = os.path.join(model_dir, 'model.sdf')
        return path if os.path.isfile(path) else None

    # ---- parsing -----------------------------------------------------------------------------
    def parse(self) -> List[Shape]:
        root = ET.parse(self.world_file).getroot()
        world = root.find('world')
        if world is None:
            raise ValueError(f'No <world> element in {self.world_file}')
        base_dir = os.path.dirname(self.world_file)
        shapes: List[Shape] = []
        for model in world.findall('model'):
            self._parse_model(model, IDENTITY, '', base_dir, shapes, None)
        for inc in world.findall('include'):
            self._parse_include(inc, IDENTITY, '', base_dir, shapes, None)
        return shapes

    def _parse_include(self, inc: ET.Element, parent_pose: Pose, prefix: str, base_dir: str,
                       shapes: List[Shape], scope: Optional[Scope]):
        uri = child_text(inc, 'uri') or ''
        path = self.resolve_uri(uri, base_dir)
        if path and os.path.isdir(path):
            path = self.model_sdf_from_dir(path)
        if not path:
            self.log.warn(f'Could not resolve <include> uri "{uri}", skipping it')
            return
        try:
            model = ET.parse(path).getroot().find('model')
        except ET.ParseError as e:
            self.log.warn(f'Failed to parse {path}: {e}')
            return
        if model is None:
            self.log.warn(f'{path} has no <model>, skipping include "{uri}"')
            return
        # <include> can override the model's name and pose
        name = child_text(inc, 'name')
        if name:
            model.set('name', name.strip())
        inc_pose = inc.find('pose')
        if inc_pose is not None:
            old = model.find('pose')
            if old is not None:
                model.remove(old)
            model.insert(0, inc_pose)
        self._parse_model(model, parent_pose, prefix, os.path.dirname(path), shapes, scope)

    def _parse_model(self, model: ET.Element, parent_pose: Pose, prefix: str, base_dir: str,
                     shapes: List[Shape], parent_scope: Optional[Scope]):
        name = model.get('name', 'model')
        if name in self.exclude:
            return
        path = f'{prefix}{name}'
        pose_elem = model.find('pose')
        model_pose = (parent_scope.resolve(pose_elem, parent_pose) if parent_scope
                      else compose(parent_pose, parse_pose_elem(pose_elem)))
        scope = Scope(model_pose)

        # First pass: compute poses of links / frames / nested models so relative_to works
        # regardless of declaration order (one level of dependency is enough for typical worlds).
        for _ in range(2):
            for tag in ('link', 'frame', 'model'):
                for e in model.findall(tag):
                    parent = scope.frames.get(e.get('attached_to', ''), model_pose) if tag == 'frame' else model_pose
                    scope.frames[e.get('name', '')] = scope.resolve(e.find('pose'), parent)

        for link in model.findall('link'):
            link_pose = scope.frames.get(link.get('name', ''), model_pose)
            self._parse_link(link, link_pose, f'{path}/', base_dir, shapes, scope)
        for nested in model.findall('model'):
            self._parse_model(nested, model_pose, f'{path}/', base_dir, shapes, scope)
        for inc in model.findall('include'):
            self._parse_include(inc, model_pose, f'{path}/', base_dir, shapes, scope)

    def _parse_link(self, link: ET.Element, link_pose: Pose, prefix: str, base_dir: str,
                    shapes: List[Shape], scope: Scope):
        link_name = link.get('name', 'link')
        tag = 'collision' if self.use_collision else 'visual'
        for vis in link.findall(tag):
            geom = vis.find('geometry')
            if geom is None:
                continue
            # A visual's pose is relative to its link unless relative_to says otherwise
            vis_pose = scope.resolve(vis.find('pose'), link_pose)
            color = self._color(vis)
            ns = f'{prefix}{link_name}/{vis.get("name", tag)}'
            shape = self._geometry(geom, vis_pose, ns, color, base_dir)
            if shape is not None:
                shapes.append(shape)

    @staticmethod
    def _color(vis: ET.Element) -> Optional[Tuple[float, float, float, float]]:
        mat = vis.find('material')
        rgba = None
        if mat is not None:
            for tag in ('diffuse', 'ambient'):
                txt = child_text(mat, tag)
                if txt:
                    v = floats(txt, [0.7, 0.7, 0.7, 1.0])
                    rgba = [v[0], v[1], v[2], v[3] if len(v) > 3 else 1.0]
                    break
        transparency = child_text(vis, 'transparency')
        if transparency is not None:
            rgba = rgba or [0.7, 0.7, 0.7, 1.0]
            rgba[3] *= max(0.0, min(1.0, 1.0 - float(transparency)))
        return tuple(rgba) if rgba else None

    def _geometry(self, geom: ET.Element, pose: Pose, ns: str, color, base_dir: str) -> Optional[Shape]:
        if (g := geom.find('box')) is not None:
            s = floats(child_text(g, 'size'), [1, 1, 1])
            return Shape(ns, Marker.CUBE, pose, (s[0], s[1], s[2]), color)

        if (g := geom.find('cylinder')) is not None:
            r = float(child_text(g, 'radius') or 1.0)
            l = float(child_text(g, 'length') or 1.0)
            return Shape(ns, Marker.CYLINDER, pose, (2 * r, 2 * r, l), color)

        if (g := geom.find('sphere')) is not None:
            r = float(child_text(g, 'radius') or 1.0)
            return Shape(ns, Marker.SPHERE, pose, (2 * r, 2 * r, 2 * r), color)

        if (g := geom.find('ellipsoid')) is not None:
            r = floats(child_text(g, 'radii'), [1, 1, 1])
            return Shape(ns, Marker.SPHERE, pose, (2 * r[0], 2 * r[1], 2 * r[2]), color)

        if (g := geom.find('capsule')) is not None:
            # Approximated as a cylinder covering the full capsule length
            r = float(child_text(g, 'radius') or 0.5)
            l = float(child_text(g, 'length') or 1.0)
            return Shape(ns, Marker.CYLINDER, pose, (2 * r, 2 * r, l + 2 * r), color)

        if (g := geom.find('plane')) is not None:
            # Drawn as a thin box whose top face sits on the plane
            normal = floats(child_text(g, 'normal'), [0, 0, 1])
            size = floats(child_text(g, 'size'), [1, 1])
            thickness = 0.01
            q = quat_from_z_to((normal[0], normal[1], normal[2]))
            plane_pose = compose(pose, ((0.0, 0.0, 0.0), q))
            plane_pose = compose(plane_pose, ((0.0, 0.0, -thickness / 2), (0.0, 0.0, 0.0, 1.0)))
            return Shape(ns, Marker.CUBE, plane_pose, (size[0], size[1], thickness), color)

        if (g := geom.find('mesh')) is not None:
            uri = (child_text(g, 'uri') or '').strip()
            scale = floats(child_text(g, 'scale'), [1, 1, 1])
            path = self.resolve_uri(uri, base_dir)
            if path:
                mesh_uri = 'file://' + os.path.abspath(path).replace('\\', '/')
            else:
                self.log.warn(f'Could not find mesh "{uri}" for {ns}; passing the URI through unchanged')
                mesh_uri = uri
            return Shape(ns, Marker.MESH_RESOURCE, pose, (scale[0], scale[1], scale[2]), color, mesh_uri)

        self.log.warn(f'Unsupported geometry in {ns} ({[c.tag for c in geom]}), skipping it')
        return None


# ----------------------------------------------------------------------------------------------
# ROS node
# ----------------------------------------------------------------------------------------------
def default_world_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('neos_sim'), 'worlds', 'hallway.sdf')
    except Exception:
        return ''


class SdfToMarkers(Node):
    def __init__(self):
        super().__init__('sdf_to_markers')
        self.declare_parameter('world_file', default_world_file())
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('topic', '/world_markers')
        self.declare_parameter('use_collision', False)
        self.declare_parameter('exclude_models', [''])
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('default_color', [0.7, 0.7, 0.7, 1.0])
        self.declare_parameter('resource_paths', [''])

        p = self.get_parameter
        world_file = p('world_file').value
        self.frame_id = p('frame_id').value
        self.default_color = list(p('default_color').value)
        exclude = [m for m in p('exclude_models').value if m]
        resource_paths = [r for r in p('resource_paths').value if r]

        if not world_file or not os.path.isfile(world_file):
            raise FileNotFoundError(f'world_file "{world_file}" does not exist')

        parser = SdfWorldParser(world_file, p('use_collision').value, exclude, resource_paths, self.get_logger())
        shapes = parser.parse()
        self.markers = self._to_markers(shapes)
        self.get_logger().info(
            f'Loaded {len(shapes)} visuals from {world_file}; publishing on {p("topic").value} in frame "{self.frame_id}"')

        # Transient local ("latched") so Foxglove/RViz get the world even if they connect later
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, p('topic').value, qos)
        self.publish()

        rate = float(p('publish_rate').value)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self.publish)

    def _to_markers(self, shapes: List[Shape]) -> List[Marker]:
        # First marker clears anything left over from a previous run / previous world
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers = [clear]

        ids: Dict[str, int] = {}
        for s in shapes:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = s.ns
            m.id = ids.setdefault(s.ns, 0)
            ids[s.ns] += 1
            m.type = s.type
            m.action = Marker.ADD
            (px, py, pz), (qx, qy, qz, qw) = s.pose
            m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, pz
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
            m.scale.x, m.scale.y, m.scale.z = s.scale
            if s.type == Marker.MESH_RESOURCE:
                m.mesh_resource = s.mesh_uri
                # No SDF material -> keep the mesh's own textures/colours
                m.mesh_use_embedded_materials = s.color is None
            rgba = s.color if s.color is not None else (
                (0.0, 0.0, 0.0, 0.0) if s.type == Marker.MESH_RESOURCE else self.default_color)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
            m.frame_locked = False
            markers.append(m)
        return markers

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for m in self.markers:
            m.header.stamp = stamp
        self.pub.publish(MarkerArray(markers=self.markers))


def main(args=None):
    rclpy.init(args=args)
    node = SdfToMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
