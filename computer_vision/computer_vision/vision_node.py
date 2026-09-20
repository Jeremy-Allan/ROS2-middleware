#!/usr/bin/env python3
"""Vision snapshot node — Kinect v2 -> object_dictionary via environment_mapping_node.

Snapshot flow:
  1. Capture N depth frames (per-pixel median) + a few colour frames (for YOLO).
     YOLO frames come from HD colour if hd_color_topic is set, else SD.
  2. Backproject to base_link; keep the workspace; fit the TABLE PLANE; keep points above it;
     mask the arm.
  3. DBSCAN cluster -> merge dropout-split clusters -> drop border-truncated clusters.
  4. YOLO on several colour frames, aggregated over time (a detection must persist).
     HD boxes are mapped back to SD pixel space via a table-plane homography.
  5. Split clusters that several boxes claim; associate boxes<->clusters (Hungarian).
       cluster + box      -> named object
       cluster, no box    -> 'object' (fallback, kept as an obstacle)
       box, no cluster    -> depth hole? place a class-prior box by ray-casting; else drop
  6. Label hysteresis vs. the previous dictionary (no flicker), stable IDs, grace period for
     objects that go unseen (arm occlusion).
  7. Push to environment_mapping_node via /update_scene_objects.

Parameters live in YAML (camera.yaml / vision.yaml / yolo.yaml); DEFAULTS below are fallbacks.
Service:  /vision/snapshot   (kinova_interfaces/srv/Snapshot)

This node never touches MoveIt. The environment node owns the dictionary + scene.
"""
import copy
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
from geometry_msgs.msg import Point
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

from kinova_interfaces.srv import GetSceneObjects, Snapshot, UpdateSceneObjects


# ---------------------------------------------------------------- config defaults
DEFAULTS = {
    # --- camera.yaml ---
    'base_frame': 'base_link',
    'color_topic': '/kinect2/sd/image_color_rect',
    'depth_topic': '/kinect2/sd/image_depth_rect',
    'info_topic': '/kinect2/sd/camera_info',
    'hd_color_topic': '/kinect2/hd/image_color_rect',   # '' = use SD for YOLO
    'hd_info_topic': '/kinect2/hd/camera_info',
    'arm_frames': ['shoulder_link', 'arm_link', 'forearm_link',
                   'lower_wrist_link', 'upper_wrist_link', 'end_effector_link'],
    'arm_radius': 0.12,
    # --- vision.yaml ---
    'workspace': [-0.6, 0.6, -0.5, 0.5],
    'table_z': 0.0,
    'plane_band': 0.08,
    'min_height': 0.006,
    'max_height': 0.30,
    'cluster_eps': 0.02,
    'cluster_min_points': 30,
    'merge_gap': 0.02,
    'drop_border_clusters': True,
    'num_frames': 8,
    'save_dir': '',
    'unknown_margin': 0.005,
    'id_match_dist': 0.08,
    'label_flip_conf': 0.6,
    'grace_snapshots': 2,
    'carried_conf_floor': 0.25,   # below this, a carried label is dropped
    # --- yolo.yaml ---
    'yolo_model': '',
    'yolo_device': '',
    'yolo_imgsz': 640,
    'yolo_conf': 0.25,
    'yolo_frames': 5,
    'yolo_min_presence': 0.6,
    'yolo_track_iou': 0.5,
    'match_thresh': 0.30,
    'claim_frac': 0.25,
    'split_by_boxes': True,
    'orphan_conf': 0.55,
    'orphan_depth_valid_frac': 0.3,
    'ignore_labels': '',
    'label_alias': '',
}

DEFAULT_CLASS_DIMS = {'cube': [0.05, 0.05, 0.05], 'pen': [0.015, 0.15, 0.012],
                      'cup': [0.08, 0.08, 0.10], 'bottle': [0.07, 0.07, 0.22],
                      'book': [0.15, 0.22, 0.02]}
DEFAULT_CLASS_SHAPE = {'bottle': 'CYLINDER', 'cup': 'CYLINDER', 'can': 'CYLINDER', 'ball': 'SPHERE'}


# ---------------------------------------------------------------- helpers

def _csv(s):
    return [x.strip().lower() for x in str(s).split(',') if x.strip()]


def _pairs(s):
    out = {}
    for item in str(s).split(','):
        if ':' in item:
            a, b = item.split(':', 1)
            out[a.strip().lower()] = b.strip().lower()
    return out


def _pval(p):
    return p.value if hasattr(p, 'value') else p


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
    """Majority-vote colour name over pixels. Tune thresholds for your lighting."""
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


def box_area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def in_box(uv, box):
    u0, v0, u1, v1 = box
    return ((uv[:, 0] >= u0) & (uv[:, 0] <= u1)
            & (uv[:, 1] >= v0) & (uv[:, 1] <= v1))


def fit_table_plane(pts, z0, band=0.04, iters=4, min_tol=0.004):
    """Robust least-squares plane  z = a*x + b*y + c  through the table surface."""
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
    """Union clusters whose nearest points are within `gap` metres, then refit each group."""
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
    x0, x1, y0, y1 = workspace
    return bool(c.pts[:, 0].min() < x0 + margin or c.pts[:, 0].max() > x1 - margin
                or c.pts[:, 1].min() < y0 + margin or c.pts[:, 1].max() > y1 - margin)


def z_peaks(pts, z0, span_below=0.05, span_above=0.25, bin_m=0.001, top=5):
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
    pts: np.ndarray
    uv: np.ndarray
    bbox2d: tuple
    center: np.ndarray
    dims: np.ndarray
    yaw: float


def aggregate_detections(per_frame, iou_thr=0.5, min_presence=0.6):
    """Turn per-frame YOLO detections into stable ones (IoU-tracked across frames)."""
    n_frames = len(per_frame)
    if n_frames == 0:
        return []
    tracks = []
    for fi, dets in enumerate(per_frame):
        used = set()
        for d in sorted(dets, key=lambda d: -d.conf):
            best, best_iou = None, iou_thr
            for ti, tr in enumerate(tracks):
                if ti in used:
                    continue
                v = box_iou(d.box, tr['ref'])
                if v > best_iou:
                    best, best_iou = ti, v
            if best is None:
                tracks.append({'ref': d.box, 'items': [(fi, d)]})
                used.add(len(tracks) - 1)
            else:
                tracks[best]['items'].append((fi, d))
                tracks[best]['ref'] = tuple(np.mean([x.box for _, x in tracks[best]['items']], axis=0))
                used.add(best)
    out = []
    for tr in tracks:
        frames = {fi for fi, _ in tr['items']}
        if len(frames) / n_frames < min_presence:
            continue
        score = {}
        for _, d in tr['items']:
            score[d.label] = score.get(d.label, 0.0) + d.conf
        label = max(score, key=score.get)
        win = [d for _, d in tr['items'] if d.label == label]
        box = tuple(float(x) for x in np.median([d.box for d in win], axis=0))
        conf = float(np.mean([d.conf for d in win])) * (len(win) / n_frames)
        out.append(Detection(label, conf, box))
    return sorted(out, key=lambda d: -d.conf)


# ---------------------------------------------------------------- node

class VisionSnapshotNode(Node):

    def __init__(self):
        super().__init__('vision_snapshot_node',
                         automatically_declare_parameters_from_overrides=True)
        for k, v in DEFAULTS.items():
            if not self.has_parameter(k):
                self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in DEFAULTS}
        self._warn_unknown_params()

        self.class_dims = {k: list(v) for k, v in DEFAULT_CLASS_DIMS.items()}
        for k, p in self.get_parameters_by_prefix('class_dims').items():
            self.class_dims[k] = [float(x) for x in _pval(p)]
        self.class_shape = dict(DEFAULT_CLASS_SHAPE)
        for k, p in self.get_parameters_by_prefix('class_shape').items():
            self.class_shape[k] = str(_pval(p)).upper()
        self.ignore_labels = set(_csv(self.cfg['ignore_labels']))
        self.label_alias = _pairs(self.cfg['label_alias'])

        self.base_frame = self.cfg['base_frame']
        self.workspace = list(self.cfg['workspace'])
        self.table_z = float(self.cfg['table_z'])
        self.plane = None
        self._mask_uv = None
        self._missing_frames = set()

        self.workspace_pub = self.create_publisher(MarkerArray, '/vision/workspace', 1)
        self.workspace_timer = self.create_timer(1.0, self.publish_workspace_markers)

        # --- ROS plumbing ---
        self.bridge = CvBridge()
        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._depths = deque(maxlen=64)
        self._colors = deque(maxlen=32)
        self._color = None
        self._info = None
        self._hd_colors = deque(maxlen=16)
        self._hd_color = None
        self._hd_info = None

        self.create_subscription(Image, self.cfg['color_topic'], self._on_color,
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(Image, self.cfg['depth_topic'], self._on_depth,
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(CameraInfo, self.cfg['info_topic'], self._on_info,
                                 qos_profile_sensor_data, callback_group=self.cb)
        if self.cfg['hd_color_topic']:
            self.create_subscription(Image, self.cfg['hd_color_topic'], self._on_hd_color,
                                     qos_profile_sensor_data, callback_group=self.cb)
            self.create_subscription(CameraInfo, self.cfg['hd_info_topic'], self._on_hd_info,
                                     qos_profile_sensor_data, callback_group=self.cb)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Kinect stream health (logged by the watchdog) ---
        self._topics = {'color': self.cfg['color_topic'], 'depth': self.cfg['depth_topic'],
                        'info': self.cfg['info_topic']}
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

        self.yolo = self._load_yolo()
        self._log_config()

    # ------------------------------------------------ setup helpers

    def _warn_unknown_params(self):
        known = set(DEFAULTS) | {'use_sim_time'}
        for name in list(self._parameters.keys()):
            if name in known or name.startswith(('class_dims.', 'class_shape.', 'qos_overrides')):
                continue
            self.get_logger().warn(f"unknown parameter '{name}' in your YAML (typo? it is ignored)")

    def _load_yolo(self):
        path = self.cfg['yolo_model']
        if not path:
            return None
        from ultralytics import YOLO
        model = YOLO(path)
        names = [self._norm_label(v) for v in model.names.values()]
        model.predict(np.zeros((424, 512, 3), np.uint8), verbose=False,
                      device=self.cfg['yolo_device'] or None)
        no_prior = [n for n in names if n not in self.class_dims]
        self.get_logger().info(f'YOLO loaded: {len(names)} classes from {path}')
        if no_prior:
            self.get_logger().info(
                f'  classes without class_dims (fine for cluster-backed objects, but a depth-hole '
                f'detection of these gets a 5 cm cube prior): {no_prior[:12]}'
                f'{" ..." if len(no_prior) > 12 else ""}')
        return model

    def _log_config(self):
        c = self.cfg
        self.get_logger().info(
            f'config: workspace={self.workspace} table_z={self.table_z} '
            f'eps={c["cluster_eps"]} merge_gap={c["merge_gap"]} min_height={c["min_height"]} | '
            f'yolo={"ON (" + c["yolo_model"] + ")" if self.yolo else "OFF (clusters only)"} | '
            f'grace={c["grace_snapshots"]}')
        self.get_logger().info('vision_snapshot_node ready (/vision/snapshot)')

    def _norm_label(self, raw):
        x = str(raw).strip().lower()
        x = self.label_alias.get(x, x)
        return x.replace(' ', '_')

    # ------------------------------------------------ subscriptions / health

    def _touch(self, name, shape=None):
        s = self._stats[name]
        s['n'] += 1
        s['t'] = time.time()
        if shape is not None:
            s['shape'] = shape

    def _on_color(self, msg):
        self._color = msg
        with self._lock:
            self._colors.append(msg)
        self._touch('color', (msg.width, msg.height, msg.encoding))

    def _on_info(self, msg):
        self._info = msg
        self._touch('info', (msg.width, msg.height, msg.header.frame_id))

    def _on_hd_color(self, msg):
        self._hd_color = msg
        with self._lock:
            self._hd_colors.append(msg)

    def _on_hd_info(self, msg):
        self._hd_info = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depths.append(msg)
        self._touch('depth', (msg.width, msg.height, msg.encoding))

    def _missing_topics(self):
        return [k for k in self._topics if self._stats[k]['n'] == 0]

    def _missing_report(self):
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
        now = time.time()
        dt = max(now - self._last_tick, 1e-6)
        hz = {k: (self._stats[k]['n'] - self._last_n[k]) / dt for k in self._topics}
        self._last_n = {k: self._stats[k]['n'] for k in self._topics}
        self._last_tick = now

        if self._missing_topics():
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
        """n depth frames -> median; a few evenly spaced colour frames from the same window.
        Returns (color_bgr, yolo_frames, depth, K_sd, cam_frame_sd, K_yolo, cam_frame_yolo)."""
        if self._missing_topics():
            raise RuntimeError('Kinect stream not up: ' + self._missing_report())
        with self._lock:
            self._depths.clear()
            self._colors.clear()
        t0 = time.time()
        while len(self._depths) < n:
            if time.time() - t0 > 10.0 + 0.3 * n or not rclpy.ok():
                raise TimeoutError(f'Timed out waiting for {n} depth frames '
                                   f'(got {len(self._depths)}); depth stream stalled?')
            time.sleep(0.02)
        with self._lock:
            dmsgs = list(self._depths)[-n:]
            cmsgs = list(self._colors)

        stack = np.stack([self._depth_to_m(m) for m in dmsgs])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            depth = np.nanmedian(stack, axis=0)
        depth[np.isfinite(stack).sum(axis=0) < n * 0.5] = np.nan

        color = self.bridge.imgmsg_to_cv2(self._color, 'bgr8')
        if color.shape[:2] != depth.shape:
            raise RuntimeError(f'colour {color.shape[:2]} and depth {depth.shape} sizes differ - '
                               f'use the registered topics')
        K = np.array(self._info.k).reshape(3, 3)

        # YOLO frames: HD if configured and available, else SD.
        k = self.cfg['yolo_frames'] if self.yolo is not None else 1
        use_hd = bool(self.cfg['hd_color_topic'] and self._hd_info is not None
                      and len(self._hd_colors))
        if use_hd:
            with self._lock:
                hd_msgs = list(self._hd_colors)
            idx = np.unique(np.linspace(0, len(hd_msgs) - 1,
                                        max(1, min(k, len(hd_msgs)))).astype(int))
            frames = [self.bridge.imgmsg_to_cv2(hd_msgs[i], 'bgr8') for i in idx]
            K_yolo = np.array(self._hd_info.k).reshape(3, 3)
            cam_yolo = self._hd_info.header.frame_id
        else:
            if not cmsgs:
                cmsgs = [self._color]
            idx = np.unique(np.linspace(0, len(cmsgs) - 1,
                                        max(1, min(k, len(cmsgs)))).astype(int))
            frames = [self.bridge.imgmsg_to_cv2(cmsgs[i], 'bgr8') for i in idx]
            K_yolo = K
            cam_yolo = self._info.header.frame_id

        return color, frames, depth, K, self._info.header.frame_id, K_yolo, cam_yolo

    def tf_matrix(self, target, source):
        t = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=2.0))
        tr, r = t.transform.translation, t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_mat((r.x, r.y, r.z, r.w))
        T[:3, 3] = (tr.x, tr.y, tr.z)
        return T

    def _frame_position(self, frame):
        if frame in self._missing_frames:
            return None
        if not self.tf_buffer.can_transform(self.base_frame, frame, Time()):
            self._missing_frames.add(frame)
            self.get_logger().warn(f"arm frame '{frame}' not in TF - skipping (fix `arm_frames`)")
            return None
        return self.tf_matrix(self.base_frame, frame)[:3, 3]

    # ------------------------------------------------ geometry

    def table_z_at(self, x, y):
        if self.plane is None:
            return self.table_z
        return float(self.plane[0] * x + self.plane[1] * y + self.plane[2])

    def raycast_plane(self, u, v, K, T, offset=0.0):
        """Ray through pixel (u, v) intersected with the table plane raised by `offset`."""
        d_cam = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        d, o = T[:3, :3] @ d_cam, T[:3, 3]
        a, b, c = self.plane if self.plane is not None else (0.0, 0.0, self.table_z)
        denom = d[2] - a * d[0] - b * d[1]
        if abs(denom) < 1e-6:
            return None
        s = (a * o[0] + b * o[1] + c + offset - o[2]) / denom
        return o + s * d if s > 0 else None

    def _project_base_to_pixels(self, pts_base, K, T_base_from_cam):
        """Project Nx3 base_link points into a camera's pixels.
        T_base_from_cam is what tf_matrix() returns (cam->base). Invert for base->cam."""
        T_cam_from_base = np.linalg.inv(T_base_from_cam)
        pts_h = np.c_[pts_base, np.ones(len(pts_base))]
        pts_cam = (T_cam_from_base @ pts_h.T).T[:, :3]
        z = pts_cam[:, 2]
        uv = np.full((len(pts_base), 2), np.nan)
        ok = z > 0.1
        uv[ok, 0] = K[0, 0] * pts_cam[ok, 0] / z[ok] + K[0, 2]
        uv[ok, 1] = K[1, 1] * pts_cam[ok, 1] / z[ok] + K[1, 2]
        return uv

    def _hd_to_sd_homography(self, K_hd, T_hd, K_sd, T_sd):
        """3x3 homography mapping HD pixels to SD pixels for points on the fitted
        table plane. Used to associate HD YOLO boxes with SD-space clusters."""
        if self.plane is None:
            return None
        a, b, c = self.plane
        x0, x1, y0, y1 = self.workspace
        pts_base = np.array([
            [x0, y0, a * x0 + b * y0 + c],
            [x1, y0, a * x1 + b * y0 + c],
            [x1, y1, a * x1 + b * y1 + c],
            [x0, y1, a * x0 + b * y1 + c],
        ], dtype=np.float64)
        uv_hd = self._project_base_to_pixels(pts_base, K_hd, T_hd)
        uv_sd = self._project_base_to_pixels(pts_base, K_sd, T_sd)
        if np.any(~np.isfinite(uv_hd)) or np.any(~np.isfinite(uv_sd)):
            return None
        H, _ = cv2.findHomography(uv_hd.astype(np.float32), uv_sd.astype(np.float32))
        return H

    @staticmethod
    def _hd_box_to_sd(box, H):
        u0, v0, u1, v1 = box
        corners = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]],
                           dtype=np.float32).reshape(-1, 1, 2)
        m = cv2.perspectiveTransform(corners, H).reshape(-1, 2)
        return (float(m[:, 0].min()), float(m[:, 1].min()),
                float(m[:, 0].max()), float(m[:, 1].max()))

    def backproject(self, depth, K, T):
        h, w = depth.shape
        v, u = np.mgrid[0:h, 0:w]
        ok = np.isfinite(depth) & (depth > 0.3) & (depth < 4.5)
        z, uu, vv = depth[ok], u[ok], v[ok]
        cam = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], axis=1)
        return cam @ T[:3, :3].T + T[:3, 3], np.stack([uu, vv], axis=1)

    def filter_workspace(self, pts, uv):
        """Workspace box -> fit table plane -> height above plane -> arm mask."""
        x0, x1, y0, y1 = self.workspace
        in_xy = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
        pts, uv = pts[in_xy], uv[in_xy]

        self.plane = fit_table_plane(pts, self.table_z, self.cfg['plane_band'])
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
            if abs(zc - self.table_z) > 0.5 * self.cfg['plane_band']:
                self.get_logger().warn(
                    f'fitted table is {zc:+.3f} m at the workspace centre but '
                    f'table_z={self.table_z:+.3f} - set table_z: {zc:.3f} in vision.yaml')
            if tilt > 1.0:
                self.get_logger().warn(
                    f'table tilt {tilt:.1f} deg vs base_link: if the robot sits on a level table '
                    f'this is camera extrinsic (roll/pitch) error')

        h = pts[:, 2] - base
        keep = (h >= self.cfg['min_height']) & (h <= self.cfg['max_height'])
        for f in self.cfg['arm_frames']:
            p = self._frame_position(f)
            if p is not None:
                keep &= np.linalg.norm(pts - p, axis=1) > self.cfg['arm_radius']

        self._mask_uv = uv[keep]
        return pts[keep], uv[keep]

    def denoise(self, pts, uv):
        """Drop points whose 10 nearest neighbours are unusually far."""
        from sklearn.neighbors import NearestNeighbors
        if len(pts) < 30:
            return pts, uv
        nn = NearestNeighbors(n_neighbors=10).fit(pts)
        d, _ = nn.kneighbors(pts)
        mean_d = d[:, 1:].mean(axis=1)
        keep = mean_d < np.percentile(mean_d, 90)
        return pts[keep], uv[keep]

    def build_cluster(self, pts, uv):
        """Oriented box (2%/98% outlier trim, minAreaRect); height above the fitted plane."""
        xlo, xhi = np.percentile(pts[:, 0], [2, 98])
        ylo, yhi = np.percentile(pts[:, 1], [2, 98])
        m = ((pts[:, 0] >= xlo) & (pts[:, 0] <= xhi) & (pts[:, 1] >= ylo) & (pts[:, 1] <= yhi))
        pts, uv = pts[m], uv[m]
        (cx, cy), (w, d), ang = cv2.minAreaRect(pts[:, :2].astype(np.float32))
        base = self.table_z_at(cx, cy)
        h = max(float(np.percentile(pts[:, 2], 95)) - base, 0.01)
        u0, v0 = uv.min(axis=0)
        u1, v1 = uv.max(axis=0)
        return Cluster(pts=pts, uv=uv, bbox2d=(float(u0), float(v0), float(u1), float(v1)),
                       center=np.array([cx, cy, base + h / 2.0]),
                       dims=np.array([max(w, 0.01), max(d, 0.01), h]), yaw=math.radians(ang))

    def cluster(self, pts, uv):
        """DBSCAN -> merge split thin objects -> drop border-truncated clusters."""
        n_min = self.cfg['cluster_min_points']
        if len(pts) < n_min:
            return []
        labels = DBSCAN(eps=self.cfg['cluster_eps'], min_samples=10).fit_predict(pts)
        out = [self.build_cluster(pts[labels == l], uv[labels == l])
               for l in sorted(set(labels) - {-1}) if (labels == l).sum() >= n_min]

        n_before = len(out)
        out = merge_close_clusters(out, self.cfg['merge_gap'], self.build_cluster)
        if len(out) != n_before:
            self.get_logger().info(f'merged {n_before} -> {len(out)} clusters (merge_gap)')

        if self.cfg['drop_border_clusters']:
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

    def detect(self, frames, H_hd_to_sd=None):
        """YOLO on each frame, aggregate over time. If H_hd_to_sd is given, boxes are
        produced in HD pixels and mapped to SD pixel coordinates at the end."""
        if self.yolo is None:
            return []
        per_frame = []
        for img in frames:
            r = self.yolo.predict(img, conf=self.cfg['yolo_conf'], imgsz=self.cfg['yolo_imgsz'],
                                  device=self.cfg['yolo_device'] or None,
                                  agnostic_nms=True, verbose=False)[0]
            dets = []
            for b in r.boxes:
                raw = str(r.names[int(b.cls)]).strip().lower()
                label = self._norm_label(raw)
                if raw in self.ignore_labels or label in self.ignore_labels:
                    continue
                dets.append(Detection(label, float(b.conf),
                                      tuple(float(x) for x in b.xyxy[0].tolist())))
            per_frame.append(dets)
        agg = aggregate_detections(per_frame, self.cfg['yolo_track_iou'],
                                   self.cfg['yolo_min_presence'])
        if H_hd_to_sd is not None:
            agg = [Detection(d.label, d.conf, self._hd_box_to_sd(d.box, H_hd_to_sd)) for d in agg]
        return agg

    def split_merged(self, clusters, dets):
        """A cluster claimed by >= 2 boxes is touching objects: split its points by box."""
        out, n_min = [], self.cfg['cluster_min_points']
        for c in clusters:
            claim = [d for d in dets if in_box(c.uv, d.box).mean() >= self.cfg['claim_frac']]
            if len(claim) < 2 or len(c.pts) < 2 * n_min:
                out.append(c)
                continue
            lab = np.full(len(c.pts), -1)
            for k, d in sorted(enumerate(claim), key=lambda kd: box_area(kd[1].box)):
                lab[(lab == -1) & in_box(c.uv, d.box)] = k
            parts = [self.build_cluster(c.pts[lab == k], c.uv[lab == k])
                     for k in range(len(claim)) if (lab == k).sum() >= n_min]
            if len(parts) >= 2:
                self.get_logger().info(
                    f'split cluster at ({c.center[0]:.2f}, {c.center[1]:.2f}) into {len(parts)} '
                    f'({[d.label for d in claim]})')
                out.extend(parts)
            else:
                out.append(c)
        return out

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
        matches = [(i, j) for i, j in zip(ri, ci) if S[i, j] >= self.cfg['match_thresh']]
        mc, md = {i for i, _ in matches}, {j for _, j in matches}
        return (matches, [i for i in range(len(clusters)) if i not in mc],
                [j for j in range(len(dets)) if j not in md])

    # ------------------------------------------------ candidates

    def _cluster_cand(self, cl, det, color):
        if det is not None:
            label, conf, src = det.label, det.conf, 'yolo'
        else:
            label, conf, src = 'object', 0.3, 'fallback'
        return {'label': label, 'conf': conf, 'src': src,
                'color': color_name(color[cl.uv[:, 1], cl.uv[:, 0]]),
                'xy': (float(cl.center[0]), float(cl.center[1])),
                'dims': [float(x) for x in cl.dims], 'yaw': float(cl.yaw),
                'base_z': float(cl.center[2] - cl.dims[2] / 2.0),
                'geom': 'cluster', 'bbox': cl.bbox2d}

    def resolve_orphan(self, d, depth, K, T, color, report):
        """A detection with no cluster under it: false positive, or something the ToF can't see."""
        hh, ww = depth.shape
        u0, u1 = [int(np.clip(x, 0, ww - 1)) for x in (d.box[0], d.box[2])]
        v0, v1 = [int(np.clip(x, 0, hh - 1)) for x in (d.box[1], d.box[3])]
        cu, cv_ = (u0 + u1) // 2, (v0 + v1) // 2
        du, dv = max((u1 - u0) // 4, 1), max((v1 - v0) // 4, 1)
        roi = depth[cv_ - dv:cv_ + dv + 1, cu - du:cu + du + 1]
        info = {'label': d.label, 'conf': round(d.conf, 2)}

        if np.isfinite(roi).mean() >= self.cfg['orphan_depth_valid_frac']:
            report['dropped'].append({**info, 'reason': 'depth is valid there but nothing rises '
                                                        'above the table (false positive)'})
            return None
        if d.conf < self.cfg['orphan_conf']:
            report['dropped'].append({**info, 'reason': 'no depth and low confidence'})
            return None
        dims = self.class_dims.get(d.label)
        if dims is None:
            dims = [0.05, 0.05, 0.05]
            report['notes'].append(f'{d.label}: no class_dims prior, assumed 5 cm cube')
        p = self.raycast_plane(cu, cv_, K, T, offset=dims[2] / 2.0)
        x0, x1, y0, y1 = self.workspace
        if p is None or not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1):
            report['dropped'].append({**info, 'reason': 'no depth and the ray misses the workspace'})
            return None
        report['notes'].append(f'{d.label}: seen but no depth -> placed by table ray + class prior')
        patch = color[cv_ - dv:cv_ + dv + 1, cu - du:cu + du + 1].reshape(-1, 3)
        return {'label': d.label, 'conf': d.conf * 0.5, 'src': 'yolo', 'color': color_name(patch),
                'xy': (float(p[0]), float(p[1])), 'dims': [float(x) for x in dims], 'yaw': 0.0,
                'base_z': self.table_z_at(p[0], p[1]), 'geom': 'ray_prior', 'bbox': d.box}

    # ------------------------------------------------ stability

    def stabilise_labels(self, cands, prev):
        """Label hysteresis + carried-label confidence floor."""
        items = {k: v for k, v in prev.items() if not v.get('pinned') and v.get('label')}
        used = set()
        for c in sorted(cands, key=lambda c: -c['conf']):
            best, bd = None, self.cfg['id_match_dist']
            for k, p in items.items():
                if k in used:
                    continue
                pp = p['pose']['position']
                dd = math.hypot(c['xy'][0] - pp['x'], c['xy'][1] - pp['y'])
                if dd < bd:
                    best, bd = k, dd
            if best is None:
                continue
            used.add(best)
            pl = items[best]['label']
            if pl == c['label'] or pl == 'object':
                continue
            if c['src'] == 'fallback' or c['conf'] < self.cfg['label_flip_conf']:
                new_conf = 0.8 * float(items[best].get('confidence', 0.5))
                if new_conf >= self.cfg['carried_conf_floor']:
                    c['label'], c['src'] = pl, 'carried'
                    c['conf'] = new_conf

    def assign_ids(self, objs, prev):
        prev_items = [(k, v) for k, v in prev.items() if not v.get('pinned')]
        pinned = {k for k, v in prev.items() if v.get('pinned')}
        pos = lambda o: np.array([o['pose']['position']['x'], o['pose']['position']['y']])
        ids, BIG = {}, 1e3
        if objs and prev_items:
            cost = np.full((len(objs), len(prev_items)), BIG)
            for i, o in enumerate(objs):
                for j, (_, p) in enumerate(prev_items):
                    if o['label'] == p.get('label'):
                        cost[i, j] = np.linalg.norm(pos(o) - pos(p))
            for i, j in zip(*linear_sum_assignment(cost)):
                if cost[i, j] < self.cfg['id_match_dist']:
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

    def carry_unseen(self, named, prev, attached, report):
        grace = int(self.cfg['grace_snapshots'])
        if grace <= 0:
            return
        new_xy = [(o['pose']['position']['x'], o['pose']['position']['y']) for o in named.values()]
        for k, p in prev.items():
            if k in named or p.get('pinned') or k in attached or not p.get('label_source'):
                continue
            pp = p['pose']['position']
            if any(math.hypot(pp['x'] - x, pp['y'] - y) < self.cfg['id_match_dist']
                   for x, y in new_xy):
                continue
            missed = int(p.get('missed', 0)) + 1
            if missed > grace:
                report['notes'].append(f'{k}: unseen {missed - 1} snapshots -> removed')
                continue
            kept = copy.deepcopy(p)
            kept['missed'], kept['stale'] = missed, True
            named[k] = kept
            report['notes'].append(f'{k}: not seen ({missed}/{grace}) -> kept')

    # ------------------------------------------------ object construction

    def _shape(self, label, w, d, h, base_z):
        t = self.class_shape.get(label, 'BOX')
        if t == 'CYLINDER':
            return 'CYLINDER', [h, (w + d) / 4.0], base_z + h / 2.0
        if t == 'SPHERE':
            r = (w + d) / 4.0
            return 'SPHERE', [r], base_z + r
        return 'BOX', [w, d, h], base_z + h / 2.0

    def _build_object(self, c):
        w, d, h = c['dims']
        infl = self.cfg['unknown_margin'] if c['src'] == 'fallback' else 0.0
        w, d, h = w + 2 * infl, d + 2 * infl, h + infl
        stype, sdims, cz = self._shape(c['label'], w, d, h, c['base_z'])
        note = {'fallback': ' (unidentified)', }.get(c['src'], '')
        if c['geom'] == 'ray_prior':
            note = ' (no depth - estimated position)'
        return {
            'label': c['label'], 'color': c['color'], 'label_source': c['src'],
            'confidence': round(float(c['conf']), 2),
            'description': f"{c['color']} {c['label']}{note}",
            'geometry_source': c['geom'],
            'graspable': bool(c['geom'] == 'cluster' and c['src'] in ('yolo', 'carried')),
            'height_clipped': bool(c['geom'] == 'cluster'
                                   and c['dims'][2] >= self.cfg['max_height'] - 0.01),
            'last_seen': time.time(), 'missed': 0, 'stale': False,
            'pose': {'position': {'x': c['xy'][0], 'y': c['xy'][1], 'z': float(cz)},
                     'orientation': euler_to_quat(0.0, 0.0, c['yaw'])},
            'shape': {'type': stype, 'dimensions': [round(float(x), 4) for x in sdims]},
            '_bbox': c['bbox'], '_geom': c['geom'],
        }

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
            return {}, []
        res = self._wait(self.get_scene_client.call_async(GetSceneObjects.Request()), 5.0)
        if res is None or not res.success:
            return {}, []
        try:
            data = json.loads(res.objects_json)
            return data.get('objects', {}), data.get('attached', [])
        except json.JSONDecodeError:
            return {}, []

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
        if self._mask_uv is not None and len(self._mask_uv):
            u, v = self._mask_uv[:, 0], self._mask_uv[:, 1]
            img[v, u] = (0.5 * img[v, u] + 0.5 * np.array([0, 255, 255])).astype(np.uint8)
        for d in dets:
            cv2.rectangle(img, tuple(int(x) for x in d.box[:2]),
                          tuple(int(x) for x in d.box[2:]), (0, 165, 255), 1)
        palette = {'yolo': (0, 200, 0), 'carried': (0, 220, 220), 'fallback': (0, 0, 255)}
        for oid, o in named.items():
            if '_bbox' not in o:
                continue
            u0, v0, u1, v1 = [int(x) for x in o['_bbox']]
            col = ((255, 0, 255) if o.get('_geom') == 'ray_prior'
                   else palette.get(o['label_source'], (0, 0, 255)))
            cv2.rectangle(img, (u0, v0), (u1, v1), col, 2)
            cv2.putText(img, f"{oid} {o['confidence']:.2f}", (u0, max(v0 - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def publish_workspace_markers(self):
        x0, x1, y0, y1 = self.workspace
        xy = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        bot = [self.table_z_at(x, y) for x, y in xy]
        top = [z + self.cfg['max_height'] for z in bot]
        corners = np.array([[x, y, z] for (x, y), z in zip(xy, bot)] +
                           [[x, y, z] for (x, y), z in zip(xy, top)])
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'workspace', 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.006
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 0.8
        for a, b in edges:
            m.points.append(Point(x=float(corners[a][0]), y=float(corners[a][1]),
                                  z=float(corners[a][2])))
            m.points.append(Point(x=float(corners[b][0]), y=float(corners[b][1]),
                                  z=float(corners[b][2])))
        arr.markers.append(m)
        t = Marker()
        t.header.frame_id = self.base_frame
        t.header.stamp = m.header.stamp
        t.ns, t.id, t.type, t.action = 'workspace', 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        t.pose.position.z = float(max(top) + 0.10)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.04
        t.color.r, t.color.g, t.color.b, t.color.a = 0.1, 0.8, 1.0, 1.0
        t.text = f'workspace {x1 - x0:.2f} x {y1 - y0:.2f} m'
        arr.markers.append(t)
        self.workspace_pub.publish(arr)

    # ------------------------------------------------ main pipeline

    def run_snapshot(self, n, remove_missing=True):
        self._mask_uv = None
        report = {'dropped': [], 'notes': []}
        color, frames, depth, K, cam_frame, K_yolo, cam_frame_yolo = self.grab(n)
        T = self.tf_matrix(self.base_frame, cam_frame)
        pts, uv = self.backproject(depth, K, T)

        self.get_logger().info(f'raw points after backproject: {len(pts)}')
        if len(pts) > 0:
            self.get_logger().info(
                f'  bounds x[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}] '
                f'y[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}] '
                f'z[{pts[:,2].min():.3f}, {pts[:,2].max():.3f}]')
            self.get_logger().info(f'  z peaks (count, z): {z_peaks(pts, self.table_z)}')

        if self.cfg['save_dir']:
            os.makedirs(self.cfg['save_dir'], exist_ok=True)
            path = os.path.join(self.cfg['save_dir'], f'{int(time.time())}.npz')
            np.savez_compressed(path, color=color, depth=depth, K=K, T=T)
            self.get_logger().info(f'saved snapshot: {path}')

        pts, uv = self.filter_workspace(pts, uv)
        self.get_logger().info(f'after workspace/plane/arm filter: {len(pts)}')

        pts, uv = self.denoise(pts, uv)
        self.get_logger().info(f'after denoise: {len(pts)}')

        # HD -> SD homography if YOLO frames came from a different camera than the depth
        H_hd_to_sd = None
        if cam_frame_yolo != cam_frame and self.plane is not None:
            T_yolo = self.tf_matrix(self.base_frame, cam_frame_yolo)
            H_hd_to_sd = self._hd_to_sd_homography(K_yolo, T_yolo, K, T)
            if H_hd_to_sd is None:
                self.get_logger().warn('HD->SD homography failed; YOLO boxes may be misaligned')
            else:
                self.get_logger().info(
                    f'YOLO on HD frames ({frames[0].shape[1]}x{frames[0].shape[0]}) '
                    f'-> mapped to SD pixels')

        dets = self.detect(frames, H_hd_to_sd)
        if self.yolo is not None:
            self.get_logger().info(f'YOLO ({len(frames)} frames): ' + (', '.join(
                f'{d.label} {d.conf:.2f}' for d in dets) or 'no stable detections'))
        clusters = self.cluster(pts, uv)
        if dets and self.cfg['split_by_boxes']:
            clusters = self.split_merged(clusters, dets)
        for i, c in enumerate(clusters):
            self.get_logger().info(
                f'cluster {i}: n={len(c.pts)} dims=({c.dims[0]*100:.1f} x '
                f'{c.dims[1]*100:.1f} x {c.dims[2]*100:.1f}) cm '
                f'center=({c.center[0]:.2f}, {c.center[1]:.2f}, {c.center[2]:.3f})')

        matches, un_c, un_d = self.associate(clusters, dets)
        cands = [self._cluster_cand(clusters[i], dets[j], color) for i, j in matches]
        cands += [self._cluster_cand(clusters[i], None, color) for i in un_c]
        for j in un_d:
            o = self.resolve_orphan(dets[j], depth, K, T, color, report)
            if o:
                cands.append(o)

        prev, attached = self.fetch_scene()
        if attached:
            report['notes'].append(f'attached (untouched): {attached}')
        self.stabilise_labels(cands, prev)
        named = self.assign_ids([self._build_object(c) for c in cands], prev)
        if remove_missing:
            self.carry_unseen(named, prev, attached, report)
        self.publish_debug(color, named, dets)
        for note in report['notes']:
            self.get_logger().info(f'  note: {note}')
        for dr in report['dropped']:
            self.get_logger().info(f'  dropped detection: {dr}')
        return named, {'num_points': int(len(pts)), 'num_clusters': len(clusters),
                       'num_detections': len(dets), **report}

    def snapshot_cb(self, req, res):
        n = int(req.num_frames) or int(self.cfg['num_frames'])
        try:
            named, report = self.run_snapshot(n, req.remove_missing)
            payload = {k: {kk: vv for kk, vv in o.items() if not kk.startswith('_')}
                       for k, o in named.items()}
            push_res, msg = self.push(payload, req.remove_missing)
            res.success = bool(push_res and push_res.success)
            res.message = msg
            res.summary_json = json.dumps({
                'objects': {k: {'label': o['label'], 'source': o['label_source'],
                                'conf': o['confidence'], 'geometry': o['geometry_source'],
                                'stale': o.get('stale', False)}
                            for k, o in payload.items()},
                'unidentified': [k for k, o in payload.items()
                                 if o['label_source'] == 'fallback'],
                **report})
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            res.success, res.message, res.summary_json = False, f'Snapshot failed: {e}', '{}'
        return res


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