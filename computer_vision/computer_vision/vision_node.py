#!/usr/bin/env python3
"""Vision snapshot node — Kinect v2 -> object_dictionary via environment_mapping_node.

Snapshot flow:
  1. Capture N depth frames, take per-pixel median (kills ToF noise + flying pixels).
  2. Backproject to base_link using camera intrinsics + TF.
  3. Keep points inside the workspace box, fit the TABLE PLANE, keep points above it
     (min_height..max_height), and mask out the arm.
  4. DBSCAN cluster, merge clusters split by depth dropouts, drop border-truncated clusters.
  5. Fit oriented bounding box (outlier-trimmed, minAreaRect on XY), heights relative to the plane.
  6. Run YOLO on the RGB image and associate detections to clusters (optional).
  7. Assign stable IDs by matching to the current dictionary (GetSceneObjects).
  8. Push to environment_mapping_node via /update_scene_objects.

Service:  /vision/snapshot   (kinova_interfaces/srv/Snapshot)

This node never touches MoveIt. The environment node owns the dictionary + scene.
"""
import json
import math
import os
import threading
import time
import traceback
import warnings
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, Image
from sklearn.cluster import DBSCAN
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from kinova_interfaces.srv import GetSceneObjects, Snapshot, UpdateSceneObjects


# ---------------------------------------------------------------- helpers

def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
        'w': cr * cp * cy + sr * sp * sy,
    }


def color_name(bgr_pixels):
    """Majority-vote colour name over cluster pixels. Tune thresholds for your lighting."""
    if len(bgr_pixels) == 0:
        return 'unknown'
    hsv = cv2.cvtColor(bgr_pixels.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_BGR2HSV)
    hsv = hsv.reshape(-1, 3).astype(int)
    h, s, v = hsv[:, 0] * 2, hsv[:, 1], hsv[:, 2]
    names = np.full(len(h), 'gray', dtype='<U8')
    chroma = (s >= 50) & (v >= 50)
    names[v < 50] = 'black'
    names[(s < 50) & (v >= 170)] = 'white'
    for lo, hi, n in [(0, 15, 'red'), (15, 40, 'orange'), (40, 70, 'yellow'),
                      (70, 170, 'green'), (170, 260, 'blue'), (260, 330, 'purple'),
                      (330, 361, 'red')]:
        names[chroma & (h >= lo) & (h < hi)] = n
    vals, cnt = np.unique(names, return_counts=True)
    return str(vals[np.argmax(cnt)])


def box_iou(a, b):
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = iw * ih
    aa = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    ab = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / (aa + ab - inter + 1e-9)


def in_box(uv, box):
    u0, v0, u1, v1 = box
    return ((uv[:, 0] >= u0) & (uv[:, 0] <= u1)
            & (uv[:, 1] >= v0) & (uv[:, 1] <= v1))


def fit_table_plane(pts, z0, band=0.04, iters=4, min_tol=0.004):
    """Robust least-squares plane  z = a*x + b*y + c  through the table surface.

    pts: Nx3 base_link points inside the workspace XY box (NOT height-filtered).
    z0 : rough table height (the table_z param).
    Returns np.array([a, b, c]) or None if there aren't enough table points.
    Objects taller than ~min_tol are trimmed out of the fit by the residual gate.
    """
    sel = np.abs(pts[:, 2] - z0) < band
    if sel.sum() < 500:
        return None
    coef = np.array([0.0, 0.0, z0])
    for _ in range(iters):
        A = np.c_[pts[sel, 0], pts[sel, 1], np.ones(int(sel.sum()))]
        coef, *_ = np.linalg.lstsq(A, pts[sel, 2], rcond=None)
        res = pts[:, 2] - (pts[:, :2] @ coef[:2] + coef[2])
        tol = max(min_tol, 2.5 * float(np.std(res[sel])))
        sel = np.abs(res) < tol
        if sel.sum() < 500:
            return None
    return coef


def merge_close_clusters(clusters, gap, build):
    """Union clusters whose nearest points are within `gap` metres, then refit each group
    with `build(pts, uv)`. Fixes thin objects (pens) split by depth dropouts.
    Trade-off: objects closer together than `gap` will be merged."""
    n = len(clusters)
    if n < 2 or gap <= 0:
        return clusters
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    trees = [cKDTree(c.pts) for c in clusters]
    for i in range(n):
        for j in range(i + 1, n):
            d, _ = trees[i].query(clusters[j].pts[::3], distance_upper_bound=gap)
            if np.isfinite(d).any():
                parent[find(j)] = find(i)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    out = []
    for idx in groups.values():
        if len(idx) == 1:
            out.append(clusters[idx[0]])
        else:
            out.append(build(np.vstack([clusters[i].pts for i in idx]),
                             np.vstack([clusters[i].uv for i in idx])))
    return out


def touches_border(c, workspace, margin=0.01):
    """True if the cluster is cut off by the workspace box (wall / table edge / etc.)."""
    x0, x1, y0, y1 = workspace
    return bool(c.pts[:, 0].min() < x0 + margin or c.pts[:, 0].max() > x1 - margin
                or c.pts[:, 1].min() < y0 + margin or c.pts[:, 1].max() > y1 - margin)


def z_peaks(pts, z0, span_below=0.05, span_above=0.25, bin_m=0.001, top=5):
    """1 mm-bin histogram around the table height: [(count, z), ...] biggest first."""
    hist, edges = np.histogram(pts[:, 2], bins=int((span_below + span_above) / bin_m),
                               range=(z0 - span_below, z0 + span_above))
    order = np.argsort(hist)[::-1][:top]
    return [(int(hist[i]), round(float(edges[i]), 3)) for i in order]


# ---------------------------------------------------------------- dataclasses

@dataclass
class Detection:
    label: str
    conf: float
    box: tuple   # u0, v0, u1, v1


@dataclass
class Cluster:
    pts: np.ndarray     # Nx3 in base_link
    uv: np.ndarray      # Nx2 pixel coords in the colour image
    bbox2d: tuple       # u0, v0, u1, v1
    center: np.ndarray  # x, y, base_z + h/2
    dims: np.ndarray    # w, d, h
    yaw: float


# ---------------------------------------------------------------- node

class VisionSnapshotNode(Node):

    def __init__(self):
        super().__init__('vision_snapshot_node')

        # --- params ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('color_topic', '/kinect2/sd/image_color_rect')
        self.declare_parameter('depth_topic', '/kinect2/sd/image_depth_rect')
        self.declare_parameter('info_topic',  '/kinect2/sd/camera_info')
        self.declare_parameter('num_frames', 15)
        self.declare_parameter('workspace', [-0.6, 0.6, -0.5, 0.5])
        self.declare_parameter('table_z', 0.0)          # rough table height; the plane fit refines it
        self.declare_parameter('plane_band', 0.08)      # +/- band around table_z used for the plane fit
        self.declare_parameter('min_height', 0.006)     # height above the fitted plane
        self.declare_parameter('max_height', 0.30)
        self.declare_parameter('cluster_eps', 0.02)
        self.declare_parameter('cluster_min_points', 30)
        self.declare_parameter('merge_gap', 0.035)      # merge clusters closer than this (0 = off)
        self.declare_parameter('drop_border_clusters', True)
        self.declare_parameter('arm_frames', ['shoulder_link', 'arm_link', 'forearm_link', 'lower_wrist_link', 'upper_wrist_link', 'end_effector_link'])
        self.declare_parameter('arm_radius', 0.12)
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('save_dir', '')

        self.base_frame = self.get_parameter('base_frame').value
        self.workspace = self.get_parameter('workspace').value
        self.workspace_pub = self.create_publisher(MarkerArray, '/vision/workspace', 1)
        self.workspace_timer = self.create_timer(1.0, self.publish_workspace_markers)
        self.table_z   = self.get_parameter('table_z').value
        self.plane = None                 # (a, b, c): table z = a*x + b*y + c, set every snapshot
        self._mask_uv = None              # pixels that survived the height mask (debug overlay)
        self._missing_frames = set()      # arm frames not in TF (warn once)

        # --- ROS plumbing ---
        self.bridge = CvBridge()
        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._depths = deque(maxlen=64)
        self._color = None
        self._info = None

        self.create_subscription(Image, self.get_parameter('color_topic').value,
                                 self._on_color, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self._on_depth, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self._on_info, qos_profile_sensor_data,
                                 callback_group=self.cb)

        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Kinect stream health (logged by the watchdog) ---
        self._topics = {'color': self.get_parameter('color_topic').value,
                        'depth': self.get_parameter('depth_topic').value,
                        'info':  self.get_parameter('info_topic').value}
        self._stats = {k: {'n': 0, 't': None, 'shape': None} for k in self._topics}
        self._stream_up = False
        self._stalled = False
        self._tf_ok = False
        self._size_warned = False
        self._last_n = {k: 0 for k in self._topics}
        self._last_tick = time.time()
        self.watchdog = self.create_timer(1.0, self._watchdog, callback_group=self.cb)

        self.get_scene_client = self.create_client(
            GetSceneObjects, '/get_scene_objects', callback_group=self.cb)
        self.update_client = self.create_client(
            UpdateSceneObjects, '/update_scene_objects', callback_group=self.cb)

        self.srv = self.create_service(
            Snapshot, '/vision/snapshot', self.snapshot_cb, callback_group=self.cb)

        # YOLO is loaded lazily so the node runs with clustering only
        self.yolo = None
        yolo_path = self.get_parameter('yolo_model').value
        if yolo_path:
            from ultralytics import YOLO  # pip install ultralytics
            self.yolo = YOLO(yolo_path)

        self.get_logger().info('vision_snapshot_node ready (/vision/snapshot)')

    # ------------------------------------------------ subscriptions

    def _touch(self, name, shape=None):
        s = self._stats[name]
        s['n'] += 1
        s['t'] = time.time()
        if shape is not None:
            s['shape'] = shape

    def _on_color(self, msg):
        self._color = msg
        self._touch('color', (msg.width, msg.height, msg.encoding))

    def _on_info(self, msg):
        self._info = msg
        self._touch('info', (msg.width, msg.height, msg.header.frame_id))

    def _on_depth(self, msg):
        with self._lock:
            self._depths.append(msg)
        self._touch('depth', (msg.width, msg.height, msg.encoding))

    def _missing_topics(self):
        return [k for k in self._topics if self._stats[k]['n'] == 0]

    def _missing_report(self):
        """Human-readable reason each silent topic is silent."""
        parts = []
        for k in self._missing_topics():
            topic = self._topics[k]
            npub = self.count_publishers(topic)
            why = ('no publishers - is the kinect2 bridge running / is the topic name right?'
                   if npub == 0 else
                   f'{npub} publisher(s) but no data received - QoS mismatch or bridge stalled?')
            parts.append(f'{topic} [{why}]')
        return '; '.join(parts)

    def _watchdog(self):
        """1 Hz: log when the Kinect stream comes up, stalls, recovers, and when TF is missing."""
        now = time.time()
        dt = max(now - self._last_tick, 1e-6)
        hz = {k: (self._stats[k]['n'] - self._last_n[k]) / dt for k in self._topics}
        self._last_n = {k: self._stats[k]['n'] for k in self._topics}
        self._last_tick = now

        missing = self._missing_topics()
        if missing:
            self._stream_up = False
            self.get_logger().warn('Waiting for Kinect topics: ' + self._missing_report(),
                                   throttle_duration_sec=5.0)
            return

        if not self._stream_up:
            self._stream_up = True
            c, d, i = (self._stats[k]['shape'] for k in ('color', 'depth', 'info'))
            self.get_logger().info(
                f'Kinect stream UP: color {c[0]}x{c[1]} ({c[2]}), depth {d[0]}x{d[1]} ({d[2]}), '
                f'camera frame "{i[2]}", ~{hz["depth"]:.0f} Hz depth')

        c, d = self._stats['color']['shape'], self._stats['depth']['shape']
        if (c[0], c[1]) != (d[0], d[1]) and not self._size_warned:
            self._size_warned = True
            self.get_logger().error(
                f'colour {c[0]}x{c[1]} and depth {d[0]}x{d[1]} differ in size - snapshots will fail. '
                f'Use the registered topics (kinect2 "sd": colour registered to depth).')

        age = now - self._stats['depth']['t']
        if age > 3.0 and not self._stalled:
            self._stalled = True
            self.get_logger().warn(f'Depth stream STALLED: no frame for {age:.1f} s')
        elif age <= 3.0 and self._stalled:
            self._stalled = False
            self.get_logger().info('Depth stream recovered')

        if not self._tf_ok:
            frame = self._stats['info']['shape'][2]
            if self.tf_buffer.can_transform(self.base_frame, frame, Time()):
                self._tf_ok = True
                self.get_logger().info(f'TF OK: {self.base_frame} <- {frame}')
            else:
                self.get_logger().warn(
                    f'No TF {self.base_frame} <- {frame}: publish your camera calibration transform',
                    throttle_duration_sec=5.0)

    def _depth_to_m(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
        if msg.encoding in ('16UC1', 'mono16'):
            d /= 1000.0
        d[d == 0] = np.nan
        return d

    # ------------------------------------------------ capture

    def grab(self, n):
        """Collect n fresh depth frames -> per-pixel median. Returns (bgr, depth, K, cam_frame)."""
        if self._missing_topics():
            raise RuntimeError('Kinect stream not up: ' + self._missing_report())
        with self._lock:
            self._depths.clear()
        t0 = time.time()
        while len(self._depths) < n:
            if time.time() - t0 > 10.0 + 0.3 * n or not rclpy.ok():
                raise TimeoutError(
                    f'Timed out waiting for {n} depth frames (got {len(self._depths)}); '
                    f'depth stream stalled?')
            time.sleep(0.02)
        with self._lock:
            msgs = list(self._depths)[-n:]

        stack = np.stack([self._depth_to_m(m) for m in msgs])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            depth = np.nanmedian(stack, axis=0)
        # unstable pixels -> hole
        depth[np.isfinite(stack).sum(axis=0) < n * 0.5] = np.nan

        color = self.bridge.imgmsg_to_cv2(self._color, 'bgr8')
        K = np.array(self._info.k).reshape(3, 3)
        return color, depth, K, self._info.header.frame_id

    def tf_matrix(self, target, source):
        t = self.tf_buffer.lookup_transform(target, source, Time(),
                                            timeout=Duration(seconds=2.0))
        tr, r = t.transform.translation, t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_mat((r.x, r.y, r.z, r.w))
        T[:3, 3] = (tr.x, tr.y, tr.z)
        return T

    def _frame_position(self, frame):
        """Position of `frame` in base_frame, or None if it isn't in TF (non-blocking)."""
        if frame in self._missing_frames:
            return None
        if not self.tf_buffer.can_transform(self.base_frame, frame, Time()):
            self._missing_frames.add(frame)
            self.get_logger().warn(f"arm frame '{frame}' not in TF - skipping (fix `arm_frames`)")
            return None
        return self.tf_matrix(self.base_frame, frame)[:3, 3]

    # ------------------------------------------------ geometry

    def table_z_at(self, x, y):
        """Table height at (x, y) from the fitted plane (falls back to the table_z param)."""
        if self.plane is None:
            return self.table_z
        return float(self.plane[0] * x + self.plane[1] * y + self.plane[2])

    def backproject(self, depth, K, T):
        h, w = depth.shape
        v, u = np.mgrid[0:h, 0:w]
        ok = np.isfinite(depth) & (depth > 0.3) & (depth < 4.5)
        z, uu, vv = depth[ok], u[ok], v[ok]
        cam = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                        (vv - K[1, 2]) * z / K[1, 1],
                        z], axis=1)
        pts = cam @ T[:3, :3].T + T[:3, 3]
        return pts, np.stack([uu, vv], axis=1)

    def filter_workspace(self, pts, uv):
        """Workspace box -> fit table plane -> height above plane -> arm mask."""
        x0, x1, y0, y1 = self.workspace
        in_xy = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1)
                 & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
        pts, uv = pts[in_xy], uv[in_xy]

        self.plane = fit_table_plane(pts, self.table_z, self.get_parameter('plane_band').value)
        if self.plane is None:
            self.get_logger().warn('table plane fit failed - falling back to constant table_z')
            base = np.full(len(pts), self.table_z)
        else:
            base = pts[:, :2] @ self.plane[:2] + self.plane[2]
            a, b = float(self.plane[0]), float(self.plane[1])
            tilt = np.degrees(np.arctan(np.hypot(a, b)))
            zc = self.table_z_at((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            self.get_logger().info(
                f'table plane: z = {a:+.4f}*x {b:+.4f}*y {self.plane[2]:+.4f} | '
                f'tilt {tilt:.2f} deg (x-slope {np.degrees(np.arctan(a)):+.2f}, '
                f'y-slope {np.degrees(np.arctan(b)):+.2f}) | height at workspace centre {zc:+.3f} m')
            band = self.get_parameter('plane_band').value
            if abs(zc - self.table_z) > 0.5 * band:
                self.get_logger().warn(
                    f'fitted table is {zc:+.3f} m at the workspace centre but table_z={self.table_z:+.3f} '
                    f'- set table_z:={zc:.3f} (and check obstacles.json uses the same table height)')
            if tilt > 1.0:
                self.get_logger().warn(
                    f'table tilt {tilt:.1f} deg vs base_link: if the robot sits on a level table this is '
                    f'camera extrinsic (roll/pitch) error - it also shifts x/y of raised objects')

        h = pts[:, 2] - base
        keep = ((h >= self.get_parameter('min_height').value)
                & (h <= self.get_parameter('max_height').value))

        # crude arm mask: sphere around each link frame (robot_self_filter is the proper fix)
        radius = self.get_parameter('arm_radius').value
        for f in self.get_parameter('arm_frames').value:
            p = self._frame_position(f)
            if p is not None:
                keep &= np.linalg.norm(pts - p, axis=1) > radius

        self._mask_uv = uv[keep]
        return pts[keep], uv[keep]

    def build_cluster(self, pts, uv):
        """Fit an oriented box. Trims the outer 2% of points along x and y before
        minAreaRect (kills flying pixels). Height is measured above the fitted table plane."""
        xlo, xhi = np.percentile(pts[:, 0], [2, 98])
        ylo, yhi = np.percentile(pts[:, 1], [2, 98])
        m = ((pts[:, 0] >= xlo) & (pts[:, 0] <= xhi)
             & (pts[:, 1] >= ylo) & (pts[:, 1] <= yhi))
        pts, uv = pts[m], uv[m]

        xy = pts[:, :2].astype(np.float32)
        (cx, cy), (w, d), ang = cv2.minAreaRect(xy)

        base = self.table_z_at(cx, cy)
        top = float(np.percentile(pts[:, 2], 95))   # ignore top-of-object outliers
        h = max(top - base, 0.01)

        u0, v0 = uv.min(axis=0)
        u1, v1 = uv.max(axis=0)
        return Cluster(
            pts=pts, uv=uv,
            bbox2d=(float(u0), float(v0), float(u1), float(v1)),
            center=np.array([cx, cy, base + h / 2.0]),
            dims=np.array([max(w, 0.01), max(d, 0.01), h]),
            yaw=math.radians(ang),
        )

    def cluster(self, pts, uv):
        """DBSCAN -> merge split thin objects -> drop border-truncated clusters."""
        n_min = self.get_parameter('cluster_min_points').value
        if len(pts) < n_min:
            return []
        labels = DBSCAN(eps=self.get_parameter('cluster_eps').value,
                        min_samples=10).fit_predict(pts)
        out = [self.build_cluster(pts[labels == l], uv[labels == l])
               for l in sorted(set(labels) - {-1})
               if (labels == l).sum() >= n_min]

        n_before = len(out)
        out = merge_close_clusters(out, self.get_parameter('merge_gap').value,
                                   self.build_cluster)
        if len(out) != n_before:
            self.get_logger().info(f'merged {n_before} -> {len(out)} clusters (merge_gap)')

        if self.get_parameter('drop_border_clusters').value:
            kept = []
            for c in out:
                if touches_border(c, self.workspace):
                    self.get_logger().info(
                        f'dropped border-truncated cluster at ({c.center[0]:.2f}, '
                        f'{c.center[1]:.2f}) n={len(c.pts)} - widen `workspace` if it is real')
                else:
                    kept.append(c)
            out = kept

        return sorted(out, key=lambda c: (c.center[1], c.center[0]))

    # ------------------------------------------------ YOLO

    def _detect(self, color_bgr):
        """Return a list of Detection. Empty when yolo_model param is ''."""
        if self.yolo is None:
            return []
        results = self.yolo.predict(
            color_bgr,
            conf=self.get_parameter('yolo_conf').value,
            agnostic_nms=True,
            verbose=False,
        )[0]
        return [Detection(results.names[int(b.cls)],
                          float(b.conf),
                          tuple(float(x) for x in b.xyxy[0].tolist()))
                for b in results.boxes]

    @staticmethod
    def _pair_score(c, d):
        cb = c.bbox2d
        iw = max(0.0, min(cb[2], d.box[2]) - max(cb[0], d.box[0]))
        ih = max(0.0, min(cb[3], d.box[3]) - max(cb[1], d.box[1]))
        if iw * ih <= 0:
            return 0.0
        return 0.5 * box_iou(cb, d.box) + 0.5 * float(in_box(c.uv, d.box).mean())

    def associate(self, clusters, dets):
        if not clusters or not dets:
            return [], list(range(len(clusters))), list(range(len(dets)))
        S = np.array([[self._pair_score(c, d) for d in dets] for c in clusters])
        ri, ci = linear_sum_assignment(-S)
        matches = [(i, j) for i, j in zip(ri, ci) if S[i, j] >= 0.30]
        mc = {i for i, _ in matches}
        md = {j for _, j in matches}
        return matches, [i for i in range(len(clusters)) if i not in mc], \
                        [j for j in range(len(dets)) if j not in md]

    # ------------------------------------------------ object construction

    def _make_object(self, cl, det, color_bgr):
        if det is not None:
            label = det.label
            conf = det.conf
            src = 'yolo'
        else:
            # colour-named fallback; replace with CLIP/VLM later
            label = 'object'
            conf = 0.3
            src = 'fallback'

        bgr = color_bgr[cl.uv[:, 1], cl.uv[:, 0]]
        cname = color_name(bgr)
        w, d, h = [float(x) for x in cl.dims]
        # inflate fallbacks so collision shape is safe
        inflate = 0.005 if src == 'fallback' else 0.0
        w, d, h = w + 2 * inflate, d + 2 * inflate, h + inflate

        base_z = float(cl.center[2] - cl.dims[2] / 2.0)   # table height under this cluster

        return {
            'label': label,
            'color': cname,
            'label_source': src,
            'confidence': round(float(conf), 2),
            'geometry_source': 'cluster',
            'graspable': src != 'fallback',
            'last_seen': time.time(),
            'pose': {
                'position': {'x': float(cl.center[0]),
                             'y': float(cl.center[1]),
                             'z': float(base_z + h / 2.0)},
                'orientation': euler_to_quat(0.0, 0.0, float(cl.yaw)),
            },
            'shape': {
                'type': 'BOX',
                'dimensions': [round(w, 4), round(d, 4), round(h, 4)],
            },
            '_bbox': cl.bbox2d,
        }

    # ------------------------------------------------ stable IDs

    def assign_ids(self, objs, prev):
        prev_items = [(k, v) for k, v in prev.items() if not v.get('pinned')]
        pinned = {k for k, v in prev.items() if v.get('pinned')}
        pos = lambda o: np.array([o['pose']['position']['x'], o['pose']['position']['y']])

        ids, BIG = {}, 1e3
        if objs and prev_items:
            cost = np.full((len(objs), len(prev_items)), BIG)
            for i, o in enumerate(objs):
                for j, (_, p) in enumerate(prev_items):
                    if o['label'] != p.get('label'):
                        continue
                    cost[i, j] = np.linalg.norm(pos(o) - pos(p))
            for i, j in zip(*linear_sum_assignment(cost)):
                if cost[i, j] < 0.08:
                    ids[i] = prev_items[j][0]

        used = set(ids.values()) | pinned
        todo = sorted((i for i in range(len(objs)) if i not in ids),
                      key=lambda i: (objs[i]['pose']['position']['y'],
                                     objs[i]['pose']['position']['x']))
        for i in todo:
            base = f"{objs[i]['color']}_{objs[i]['label']}"
            name, k = base, 0
            while name in used:
                k += 1
                name = f'{base}_{k}'
            ids[i] = name
            used.add(name)

        return {ids[i]: objs[i] for i in range(len(objs))}

    # ------------------------------------------------ service plumbing

    def _wait(self, fut, timeout):
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def fetch_scene(self):
        if not self.get_scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/get_scene_objects not available')
            return {}
        res = self._wait(self.get_scene_client.call_async(GetSceneObjects.Request()), 5.0)
        if res is None or not res.success:
            return {}
        try:
            data = json.loads(res.objects_json)
            return data.get('objects', {})
        except json.JSONDecodeError:
            return {}

    def push(self, objects, remove_missing):
        if not self.update_client.wait_for_service(timeout_sec=5.0):
            return None, '/update_scene_objects not available'
        req = UpdateSceneObjects.Request()
        req.objects_json = json.dumps({'objects': objects})
        req.remove_missing = remove_missing
        res = self._wait(self.update_client.call_async(req), 15.0)
        return res, (res.message if res else 'timed out')

    # ------------------------------------------------ debug

    def publish_debug(self, color, named, dets):
        img = color.copy()

        # tint the pixels that survived the height mask: shows what the clusterer actually sees
        if self._mask_uv is not None and len(self._mask_uv):
            u, v = self._mask_uv[:, 0], self._mask_uv[:, 1]
            img[v, u] = (0.5 * img[v, u] + 0.5 * np.array([0, 255, 255])).astype(np.uint8)

        for d in dets:
            cv2.rectangle(img, tuple(int(x) for x in d.box[:2]),
                          tuple(int(x) for x in d.box[2:]), (0, 165, 255), 1)
        for oid, o in named.items():
            u0, v0, u1, v1 = [int(x) for x in o['_bbox']]
            col = (0, 200, 0) if o['label_source'] == 'yolo' else (0, 0, 255)
            cv2.rectangle(img, (u0, v0), (u1, v1), col, 2)
            cv2.putText(img, oid, (u0, max(v0 - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    # ------------------------------------------------ main pipeline

    def run_snapshot(self, n):
        self._mask_uv = None
        color, depth, K, cam_frame = self.grab(n)
        T = self.tf_matrix(self.base_frame, cam_frame)
        pts, uv = self.backproject(depth, K, T)

        self.get_logger().info(f'raw points after backproject: {len(pts)}')
        if len(pts) > 0:
            self.get_logger().info(
                f'  bounds x[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}] '
                f'y[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}] '
                f'z[{pts[:,2].min():.3f}, {pts[:,2].max():.3f}]')
            # 1 mm bins around table_z: the biggest peak should sit at your table surface
            self.get_logger().info(f'  z peaks (count, z): {z_peaks(pts, self.table_z)}')

        # optional: save raw frames for offline replay / tuning
        save_dir = self.get_parameter('save_dir').value
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            path = os.path.join(save_dir, f'{int(time.time())}.npz')
            np.savez_compressed(path, color=color, depth=depth, K=K, T=T)
            self.get_logger().info(f'saved snapshot: {path}')

        pts, uv = self.filter_workspace(pts, uv)
        self.get_logger().info(f'after workspace/plane/arm filter: {len(pts)}')

        dets = self._detect(color)               # [] until YOLO is enabled
        clusters = self.cluster(pts, uv)

        # per-cluster diagnostics: dims in cm
        for i, c in enumerate(clusters):
            self.get_logger().info(
                f'cluster {i}: n={len(c.pts)} '
                f'dims=({c.dims[0]*100:.1f} x {c.dims[1]*100:.1f} x {c.dims[2]*100:.1f}) cm '
                f'center=({c.center[0]:.2f}, {c.center[1]:.2f}, {c.center[2]:.3f})')

        matches, un_c, un_d = self.associate(clusters, dets)

        objs = [self._make_object(clusters[i], dets[j], color) for i, j in matches]
        objs += [self._make_object(clusters[i], None, color) for i in un_c]

        # YOLO-only detections (no cluster): dropped for now. When YOLO is on, add a
        # class-prior fallback here if you want dark/shiny objects the ToF can't see.
        _ = un_d

        prev = self.fetch_scene()
        named = self.assign_ids(objs, prev)
        self.publish_debug(color, named, dets)
        return named, {'num_points': int(len(pts)),
                       'num_clusters': len(clusters),
                       'num_detections': len(dets)}

    def snapshot_cb(self, req, res):
        n = int(req.num_frames) or self.get_parameter('num_frames').value
        try:
            named, report = self.run_snapshot(n)
            payload = {k: {kk: vv for kk, vv in o.items() if not kk.startswith('_')}
                       for k, o in named.items()}
            push_res, msg = self.push(payload, req.remove_missing)

            res.success = bool(push_res and push_res.success)
            res.message = msg
            res.summary_json = json.dumps({
                'objects': {k: {'label': o['label'],
                                'source': o['label_source'],
                                'conf': o['confidence'],
                                'geometry': o['geometry_source']}
                            for k, o in payload.items()},
                'unidentified': [k for k, o in payload.items()
                                 if o['label_source'] == 'fallback'],
                **report,
            })
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            res.success = False
            res.message = f'Snapshot failed: {e}'
            res.summary_json = '{}'
        return res
    def publish_workspace_markers(self):
        """Draw the workspace box in base_link: flat rectangle at table_z + 4 vertical edges."""
        x0, x1, y0, y1 = self.workspace
        z_bot = self.table_z
        z_top = self.table_z + self.get_parameter('max_height').value

        # 8 corners of the box
        corners = np.array([
            [x0, y0, z_bot], [x1, y0, z_bot], [x1, y1, z_bot], [x0, y1, z_bot],
            [x0, y0, z_top], [x1, y0, z_top], [x1, y1, z_top], [x0, y1, z_top],
        ])

        # 12 edges of a cube, as pairs of corner indices
        edges = [
            (0,1), (1,2), (2,3), (3,0),   # bottom square
            (4,5), (5,6), (6,7), (7,4),   # top square
            (0,4), (1,5), (2,6), (3,7),   # vertical pillars
        ]

        arr = MarkerArray()

        # 1. Wireframe box: one LINE_LIST marker, 24 points total (12 edges x 2 ends)
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'workspace'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.006                    # line width in meters
        m.color.r = 0.1
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.8
        for a, b in edges:
            m.points.append(Point(x=float(corners[a][0]), y=float(corners[a][1]), z=float(corners[a][2])))
            m.points.append(Point(x=float(corners[b][0]), y=float(corners[b][1]), z=float(corners[b][2])))
        m.lifetime.sec = 0                  # 0 = never expires
        arr.markers.append(m)

        # 2. Label floating above the box
        t = Marker()
        t.header.frame_id = self.base_frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns = 'workspace'
        t.id = 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = (x0 + x1) / 2.0
        t.pose.position.y = (y0 + y1) / 2.0
        t.pose.position.z = z_top + 0.10
        t.pose.orientation.w = 1.0
        t.scale.z = 0.04
        t.color.r = 0.1
        t.color.g = 0.8
        t.color.b = 1.0
        t.color.a = 1.0
        t.text = f'workspace {x1-x0:.2f} x {y1-y0:.2f} m'
        t.lifetime.sec = 0
        arr.markers.append(t)

        self.workspace_pub.publish(arr)




def main():
    rclpy.init()
    node = VisionSnapshotNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()