#!/usr/bin/env python3
"""Vision snapshot node - Kinect v2 -> object dictionary via environment_mapping_node.

Snapshot flow:
  1. N depth frames (per-pixel median) + a few colour frames for YOLO (HD if configured).
  2. Backproject to base_link, keep workspace, fit table plane, keep points above it, mask arm.
  3. DBSCAN -> merge dropout-split clusters -> drop border-truncated clusters.
  4. YOLO on several frames, aggregated over time (HD boxes mapped to SD via table homography).
  5. Split clusters claimed by several boxes; Hungarian cluster<->box association:
       cluster + box -> named object   |   cluster only -> 'object' (fallback obstacle)
       box only      -> dropped
  6. Label hysteresis, stable IDs, grace period for briefly unseen objects.
  7. Push to environment_mapping_node via /update_scene_objects.

Service: /vision/snapshot (kinova_interfaces/srv/Snapshot)
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
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.optimize import linear_sum_assignment
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from sklearn.cluster import DBSCAN
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kinova_interfaces.srv import GetSceneObjects, Snapshot, UpdateSceneObjects

# Fallbacks only - real values come from camera.yaml / vision.yaml / yolo.yaml
DEFAULTS = {
    # camera
    'base_frame': 'base_link',
    'color_topic': '/kinect2/sd/image_color_rect',
    'depth_topic': '/kinect2/sd/image_depth_rect',
    'info_topic': '/kinect2/sd/camera_info',
    'hd_color_topic': '/kinect2/hd/image_color_rect',   # '' = use SD for YOLO
    'hd_info_topic': '/kinect2/hd/camera_info',
    'arm_frames': ['shoulder_link', 'arm_link', 'forearm_link',
                   'lower_wrist_link', 'upper_wrist_link', 'end_effector_link'],
    'arm_radius': 0.12,
    'exclude_boxes': [0.0] * 6,
    # vision
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
    'carried_conf_floor': 0.25,
    # yolo
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
    'ignore_labels': '',
}


# ---------------------------------------------------------------- helpers

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


def box_area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def box_iou(a, b):
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = iw * ih
    return inter / (box_area(a) + box_area(b) - inter + 1e-9)


def in_box(uv, box):
    return ((uv[:, 0] >= box[0]) & (uv[:, 0] <= box[2])
            & (uv[:, 1] >= box[1]) & (uv[:, 1] <= box[3]))


def fit_table_plane(pts, z0, band=0.04, iters=4, min_tol=0.004):
    """Robust least-squares plane z = a*x + b*y + c through the table surface."""
    sel = np.abs(pts[:, 2] - z0) < band
    if sel.sum() < 500:
        return None
    coef = np.array([0.0, 0.0, z0])
    for _ in range(iters):
        A = np.c_[pts[sel, :2], np.ones(int(sel.sum()))]
        coef, *_ = np.linalg.lstsq(A, pts[sel, 2], rcond=None)
        res = pts[:, 2] - (pts[:, :2] @ coef[:2] + coef[2])
        sel = np.abs(res) < max(min_tol, 2.5 * float(np.std(res[sel])))
        if sel.sum() < 500:
            return None
    return coef


def merge_close_clusters(clusters, gap, build):
    """Union clusters whose nearest points are within `gap` metres, then refit each group."""
    n = len(clusters)
    if n < 2 or gap <= 0:
        return clusters
    trees = [cKDTree(c.pts) for c in clusters]
    adj = np.zeros((n, n), bool)
    for i in range(n):
        for j in range(i + 1, n):
            d, _ = trees[i].query(clusters[j].pts[::3], distance_upper_bound=gap)
            adj[i, j] = np.isfinite(d).any()
    k, lab = connected_components(adj, directed=False)
    out = []
    for g in range(k):
        idx = np.flatnonzero(lab == g)
        out.append(clusters[idx[0]] if len(idx) == 1 else
                   build(np.vstack([clusters[i].pts for i in idx]),
                         np.vstack([clusters[i].uv for i in idx])))
    return out


def touches_border(c, ws, margin=0.01):
    x0, x1, y0, y1 = ws
    return bool(c.pts[:, 0].min() < x0 + margin or c.pts[:, 0].max() > x1 - margin
                or c.pts[:, 1].min() < y0 + margin or c.pts[:, 1].max() > y1 - margin)


def pose_dict(x, y, z, yaw=0.0):
    qx, qy, qz, qw = Rotation.from_euler('z', yaw).as_quat()
    return {'position': {'x': float(x), 'y': float(y), 'z': float(z)},
            'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}}


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
    """Per-frame YOLO detections -> stable ones (IoU-tracked across frames)."""
    n_frames = len(per_frame)
    tracks = []
    for fi, dets in enumerate(per_frame):
        used = set()
        for d in sorted(dets, key=lambda d: -d.conf):
            best, best_iou = None, iou_thr
            for ti, tr in enumerate(tracks):
                v = box_iou(d.box, tr['ref']) if ti not in used else 0.0
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
        if len({fi for fi, _ in tr['items']}) / max(n_frames, 1) < min_presence:
            continue
        score = {}
        for _, d in tr['items']:
            score[d.label] = score.get(d.label, 0.0) + d.conf
        label = max(score, key=score.get)
        win = [d for _, d in tr['items'] if d.label == label]
        out.append(Detection(label,
                             float(np.mean([d.conf for d in win])) * len(win) / n_frames,
                             tuple(float(x) for x in np.median([d.box for d in win], axis=0))))
    return sorted(out, key=lambda d: -d.conf)


class Params:
    """Live parameter view: cfg['x'] always reflects the current `ros2 param set`."""
    def __init__(self, node):
        self.node = node

    def __getitem__(self, k):
        return self.node.get_parameter(k).value


# ---------------------------------------------------------------- node

class VisionSnapshotNode(Node):

    def __init__(self):
        super().__init__('vision_snapshot_node')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.add_on_set_parameters_callback(self._validate)
        self.cfg = Params(self)

        self.ignore_labels = {x.strip().lower() for x in self.cfg['ignore_labels'].split(',')
                              if x.strip()}
        self.base_frame = self.cfg['base_frame']
        self.plane = None
        self._mask_uv = None
        self._missing_frames = set()
        self._missed = {}          # id -> consecutive snapshots unseen (node-side only)

        self.bridge = CvBridge()
        self.cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._depths, self._colors, self._hd_colors = deque(maxlen=64), deque(maxlen=32), deque(maxlen=16)
        self._info = self._hd_info = None
        self._seen = set()

        def sub(msg_type, topic, cb):
            self.create_subscription(msg_type, topic, cb, qos_profile_sensor_data,
                                     callback_group=self.cbg)
        sub(Image, self.cfg['color_topic'], lambda m: self._on_img(self._colors, 'color', m))
        sub(Image, self.cfg['depth_topic'], lambda m: self._on_img(self._depths, 'depth', m))
        sub(CameraInfo, self.cfg['info_topic'], self._on_info)
        if self.cfg['hd_color_topic']:
            sub(Image, self.cfg['hd_color_topic'], lambda m: self._on_img(self._hd_colors, 'hd', m))
            sub(CameraInfo, self.cfg['hd_info_topic'], self._on_hd_info)

        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 1)
        self.workspace_pub = self.create_publisher(MarkerArray, '/vision/workspace', 1)
        self.create_timer(1.0, self.publish_workspace_marker)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_scene_client = self.create_client(
            GetSceneObjects, '/get_scene_objects', callback_group=self.cbg)
        self.update_client = self.create_client(
            UpdateSceneObjects, '/update_scene_objects', callback_group=self.cbg)
        self.create_service(Snapshot, '/vision/snapshot', self.snapshot_cb,
                            callback_group=self.cbg)

        self.yolo = self._load_yolo()
        self.get_logger().info(
            f'ready | workspace={self.workspace} table_z={self.table_z} | '
            f'yolo={"ON" if self.yolo else "OFF (clusters only)"}')

    # ------------------------------------------------ params / setup

    @property
    def workspace(self):
        return list(self.cfg['workspace'])

    @property
    def table_z(self):
        return float(self.cfg['table_z'])

    @staticmethod
    def _validate(params):
        for p in params:
            if p.name == 'workspace' and len(p.value) != 4:
                return SetParametersResult(
                    successful=False, reason='workspace = [x_min, x_max, y_min, y_max]')
        return SetParametersResult(successful=True)

    def _load_yolo(self):
        if not self.cfg['yolo_model']:
            return None
        from ultralytics import YOLO
        model = YOLO(self.cfg['yolo_model'])
        model.predict(np.zeros((424, 512, 3), np.uint8), verbose=False,
                      device=self.cfg['yolo_device'] or None)   # warm-up
        self.get_logger().info(f'YOLO loaded: {len(model.names)} classes')
        return model

    # ------------------------------------------------ subscriptions

    def _on_img(self, buf, name, msg):
        with self._lock:
            buf.append(msg)
        self._seen.add(name)

    def _on_info(self, msg):
        self._info = msg
        self._seen.add('info')

    def _on_hd_info(self, msg):
        self._hd_info = msg

    def _depth_to_m(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
        if msg.encoding in ('16UC1', 'mono16'):
            d /= 1000.0
        d[d == 0] = np.nan
        return d

    # ------------------------------------------------ capture

    def _pick(self, msgs, k):
        idx = np.unique(np.linspace(0, len(msgs) - 1, max(1, min(k, len(msgs)))).astype(int))
        return [self.bridge.imgmsg_to_cv2(msgs[i], 'bgr8') for i in idx]

    def grab(self, n):
        """Returns (color, yolo_frames, depth, K, cam_frame, K_hd|None, hd_frame|None)."""
        missing = [t for t, k in ((self.cfg['color_topic'], 'color'),
                                  (self.cfg['depth_topic'], 'depth'),
                                  (self.cfg['info_topic'], 'info')) if k not in self._seen]
        if missing:
            raise RuntimeError(f'No data yet on: {missing} (bridge running? topic names? QoS?)')
        with self._lock:
            self._depths.clear()
            self._colors.clear()
            self._hd_colors.clear()
        t0 = time.time()
        while len(self._depths) < n:
            if time.time() - t0 > 10.0 + 0.3 * n or not rclpy.ok():
                raise TimeoutError(f'Timed out waiting for {n} depth frames (stalled?)')
            time.sleep(0.02)
        with self._lock:
            dmsgs, cmsgs, hd = list(self._depths)[-n:], list(self._colors), list(self._hd_colors)
        if not cmsgs:
            raise RuntimeError('no colour frames received')

        stack = np.stack([self._depth_to_m(m) for m in dmsgs])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            depth = np.nanmedian(stack, axis=0)
        depth[np.isfinite(stack).sum(axis=0) < n * 0.5] = np.nan

        color = self.bridge.imgmsg_to_cv2(cmsgs[-1], 'bgr8')
        if color.shape[:2] != depth.shape:
            raise RuntimeError(f'colour {color.shape[:2]} vs depth {depth.shape} sizes differ - '
                               f'use the registered (sd) topics')
        K = np.array(self._info.k).reshape(3, 3)
        cam = self._info.header.frame_id

        if self.yolo is None:
            return color, [], depth, K, cam, None, None
        k = self.cfg['yolo_frames']
        if hd and self._hd_info is not None:
            return (color, self._pick(hd, k), depth, K, cam,
                    np.array(self._hd_info.k).reshape(3, 3), self._hd_info.header.frame_id)
        return color, self._pick(cmsgs, k), depth, K, cam, None, None

    def tf_matrix(self, target, source):
        t = self.tf_buffer.lookup_transform(target, source, Time(),
                                            timeout=Duration(seconds=2.0)).transform
        T = np.eye(4)
        r, tr = t.rotation, t.translation
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [tr.x, tr.y, tr.z]
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

    @staticmethod
    def _project(pts_base, K, T_cam_to_base):
        cam = (np.linalg.inv(T_cam_to_base) @ np.c_[pts_base, np.ones(len(pts_base))].T).T[:, :3]
        if (cam[:, 2] < 0.1).any():
            return None
        return np.c_[K[0, 0] * cam[:, 0] / cam[:, 2] + K[0, 2],
                     K[1, 1] * cam[:, 1] / cam[:, 2] + K[1, 2]].astype(np.float32)

    def _hd_to_sd_homography(self, K_hd, T_hd, K_sd, T_sd):
        """HD pixels -> SD pixels for points on the fitted table plane (workspace corners)."""
        a, b, c = self.plane
        x0, x1, y0, y1 = self.workspace
        corners = np.array([[x, y, a * x + b * y + c]
                            for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))])
        hd, sd = self._project(corners, K_hd, T_hd), self._project(corners, K_sd, T_sd)
        if hd is None or sd is None:
            return None
        return cv2.getPerspectiveTransform(hd, sd)

    @staticmethod
    def _hd_box_to_sd(box, H):
        u0, v0, u1, v1 = box
        c = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], np.float32).reshape(-1, 1, 2)
        m = cv2.perspectiveTransform(c, H).reshape(-1, 2)
        return (float(m[:, 0].min()), float(m[:, 1].min()),
                float(m[:, 0].max()), float(m[:, 1].max()))

    def backproject(self, depth, K, T):
        v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        ok = np.isfinite(depth) & (depth > 0.3) & (depth < 4.5)
        z, uu, vv = depth[ok], u[ok], v[ok]
        cam = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], axis=1)
        return cam @ T[:3, :3].T + T[:3, 3], np.stack([uu, vv], axis=1)

    def filter_workspace(self, pts, uv):
        """Workspace box -> table plane fit -> height above plane -> arm mask."""
        x0, x1, y0, y1 = self.workspace
        m = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
        pts, uv = pts[m], uv[m]

        self.plane = fit_table_plane(pts, self.table_z, self.cfg['plane_band'])
        if self.plane is None:
            self.get_logger().warn('table plane fit failed - using constant table_z')
            base = np.full(len(pts), self.table_z)
        else:
            base = pts[:, :2] @ self.plane[:2] + self.plane[2]
            tilt = math.degrees(math.atan(math.hypot(*self.plane[:2])))
            zc = self.table_z_at((x0 + x1) / 2, (y0 + y1) / 2)
            self.get_logger().info(f'table plane: tilt {tilt:.2f} deg, z at centre {zc:+.3f} m')
            if abs(zc - self.table_z) > 0.5 * self.cfg['plane_band']:
                self.get_logger().warn(f'fitted table {zc:+.3f} m but table_z={self.table_z:+.3f} '
                                       f'- set table_z: {zc:.3f}')
            if tilt > 1.0:
                self.get_logger().warn(f'table tilt {tilt:.1f} deg: likely camera extrinsic error')

        h = pts[:, 2] - base
        keep = (h >= self.cfg['min_height']) & (h <= self.cfg['max_height'])
        for f in self.cfg['arm_frames']:
            p = self._frame_position(f)
            if p is not None:
                keep &= np.linalg.norm(pts - p, axis=1) > self.cfg['arm_radius']
        for bx0, bx1, by0, by1, bz0, bz1 in np.array(self.cfg['exclude_boxes'], float).reshape(-1, 6):
            keep &= ~((pts[:, 0] >= bx0) & (pts[:, 0] <= bx1)
                      & (pts[:, 1] >= by0) & (pts[:, 1] <= by1)
                      & (pts[:, 2] >= bz0) & (pts[:, 2] <= bz1))
        self._mask_uv = uv[keep]
        return pts[keep], uv[keep]

    @staticmethod
    def denoise(pts, uv):
        """Drop points whose 10 nearest neighbours are unusually far."""
        if len(pts) < 30:
            return pts, uv
        d, _ = cKDTree(pts).query(pts, k=10)
        mean_d = d[:, 1:].mean(axis=1)
        keep = mean_d < np.percentile(mean_d, 90)
        return pts[keep], uv[keep]

    def build_cluster(self, pts, uv):
        """Oriented box (2/98% trim, minAreaRect); height above the fitted plane."""
        xlo, xhi = np.percentile(pts[:, 0], [2, 98])
        ylo, yhi = np.percentile(pts[:, 1], [2, 98])
        m = (pts[:, 0] >= xlo) & (pts[:, 0] <= xhi) & (pts[:, 1] >= ylo) & (pts[:, 1] <= yhi)
        pts, uv = pts[m], uv[m]
        (cx, cy), (w, d), ang = cv2.minAreaRect(pts[:, :2].astype(np.float32))
        base = self.table_z_at(cx, cy)
        h = max(float(np.percentile(pts[:, 2], 95)) - base, 0.01)
        (u0, v0), (u1, v1) = uv.min(axis=0), uv.max(axis=0)
        return Cluster(pts, uv, (float(u0), float(v0), float(u1), float(v1)),
                       np.array([cx, cy, base + h / 2.0]),
                       np.array([max(w, 0.01), max(d, 0.01), h]), math.radians(ang))

    def cluster(self, pts, uv):
        n_min = self.cfg['cluster_min_points']
        if len(pts) < n_min:
            return []
        labels = DBSCAN(eps=self.cfg['cluster_eps'], min_samples=10).fit_predict(pts)
        out = [self.build_cluster(pts[labels == l], uv[labels == l])
               for l in sorted(set(labels) - {-1}) if (labels == l).sum() >= n_min]
        out = merge_close_clusters(out, self.cfg['merge_gap'], self.build_cluster)
        if self.cfg['drop_border_clusters']:
            ws = self.workspace
            kept = [c for c in out if not touches_border(c, ws)]
            if len(kept) != len(out):
                self.get_logger().info(f'dropped {len(out) - len(kept)} border-truncated '
                                       f'cluster(s) - widen `workspace` if real')
            out = kept
        return sorted(out, key=lambda c: (c.center[1], c.center[0]))

    # ------------------------------------------------ YOLO

    def detect(self, frames, H_hd_to_sd=None):
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
                label = raw.replace(' ', '_')
                if raw in self.ignore_labels or label in self.ignore_labels:
                    continue
                dets.append(Detection(label, float(b.conf), tuple(b.xyxy[0].tolist())))
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
                self.get_logger().info(f'split cluster into {len(parts)}: '
                                       f'{[d.label for d in claim]}')
                out.extend(parts)
            else:
                out.append(c)
        return out

    @staticmethod
    def _pair_score(c, d):
        cb = c.bbox2d
        if min(cb[2], d.box[2]) <= max(cb[0], d.box[0]) or min(cb[3], d.box[3]) <= max(cb[1], d.box[1]):
            return 0.0
        return 0.5 * box_iou(cb, d.box) + 0.5 * float(in_box(c.uv, d.box).mean())

    def associate(self, clusters, dets):
        """Returns matches [(cluster_i, det_j)], unmatched cluster idx (boxes with no cluster are dropped)."""
        if not clusters or not dets:
            return [], list(range(len(clusters)))
        S = np.array([[self._pair_score(c, d) for d in dets] for c in clusters])
        ri, ci = linear_sum_assignment(-S)
        matches = [(i, j) for i, j in zip(ri, ci) if S[i, j] >= self.cfg['match_thresh']]
        matched = {i for i, _ in matches}
        return matches, [i for i in range(len(clusters)) if i not in matched]

    # ------------------------------------------------ objects

    def _make_object(self, cl, det, color):
        """Cluster (+ optional detection) -> scene object dict (BOX only)."""
        fallback = det is None
        label, src, conf = ('object', 'fallback', 0.3) if fallback else (det.label, 'yolo', det.conf)
        m = self.cfg['unknown_margin'] if fallback else 0.0
        w, d, h = cl.dims[0] + 2 * m, cl.dims[1] + 2 * m, cl.dims[2] + m
        base = cl.center[2] - cl.dims[2] / 2.0
        return {
            'label': label, 'label_source': src, 'confidence': round(float(conf), 2),
            'color': color_name(color[cl.uv[:, 1], cl.uv[:, 0]]),
            'pose': pose_dict(cl.center[0], cl.center[1], base + h / 2.0, cl.yaw),
            'shape': {'type': 'BOX', 'dimensions': [round(float(x), 4) for x in (w, d, h)]},
            '_bbox': cl.bbox2d,
        }

    def _make_table_object(self):
        """Axis-aligned BOX whose top sits at the highest plane point over the workspace."""
        if self.plane is None:
            return None
        a, b, c = self.plane
        x0, x1, y0, y1 = self.workspace
        t = 0.05
        top = max(a * x + b * y + c for x in (x0, x1) for y in (y0, y1))
        return {'label': 'table', 'label_source': 'vision', 'confidence': 1.0, 'color': 'unknown',
                'description': f'auto-fit table plane (top z={top:+.3f})',
                'pose': pose_dict((x0 + x1) / 2, (y0 + y1) / 2, top - t / 2),
                'shape': {'type': 'BOX', 'dimensions': [x1 - x0, y1 - y0, t]}}

    @staticmethod
    def _xy(o):
        p = o['pose']['position']
        return np.array([p['x'], p['y']])

    def stabilise_labels(self, objs, prev):
        """Label hysteresis: keep the previous label when the new one is weak or a fallback."""
        items = {k: v for k, v in prev.items() if not v.get('pinned') and v.get('label')}
        used = set()
        for o in sorted(objs, key=lambda o: -o['confidence']):
            best, bd = None, self.cfg['id_match_dist']
            for k, p in items.items():
                dd = np.linalg.norm(self._xy(o) - self._xy(p)) if k not in used else bd
                if dd < bd:
                    best, bd = k, dd
            if best is None:
                continue
            used.add(best)
            pl = items[best]['label']
            if pl in (o['label'], 'object'):
                continue
            if o['label_source'] == 'fallback' or o['confidence'] < self.cfg['label_flip_conf']:
                conf = 0.8 * float(items[best].get('confidence', 0.5))
                if conf >= self.cfg['carried_conf_floor']:
                    o.update(label=pl, label_source='carried', confidence=round(conf, 2))

    def assign_ids(self, objs, prev):
        prev_items = [(k, v) for k, v in prev.items() if not v.get('pinned')]
        pinned = {k for k, v in prev.items() if v.get('pinned')}
        ids, BIG = {}, 1e3
        if objs and prev_items:
            cost = np.full((len(objs), len(prev_items)), BIG)
            for i, o in enumerate(objs):
                for j, (_, p) in enumerate(prev_items):
                    if o['label'] == p.get('label'):
                        cost[i, j] = np.linalg.norm(self._xy(o) - self._xy(p))
            for i, j in zip(*linear_sum_assignment(cost)):
                if cost[i, j] < self.cfg['id_match_dist']:
                    ids[i] = prev_items[j][0]
        used = set(ids.values()) | pinned
        for i in sorted((i for i in range(len(objs)) if i not in ids),
                        key=lambda i: tuple(self._xy(objs[i])[::-1])):
            base = f"{objs[i]['color']}_{objs[i]['label']}"
            name, k = base, 0
            while name in used:
                k += 1
                name = f'{base}_{k}'
            ids[i] = name
            used.add(name)
        return {ids[i]: objs[i] for i in range(len(objs))}

    def carry_unseen(self, named, prev, attached, notes):
        """Keep recently-seen objects (e.g. hidden by the arm) inside the workspace for a few snapshots."""
        grace = int(self.cfg['grace_snapshots'])
        x0, x1, y0, y1 = self.workspace
        new_xy = [self._xy(o) for o in named.values()]
        missed = {}
        for k, p in prev.items():
            if k in named or k in attached or p.get('pinned') or not p.get('label_source'):
                continue
            xy = self._xy(p)
            if not (x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1):
                continue
            if any(np.linalg.norm(xy - q) < self.cfg['id_match_dist'] for q in new_xy):
                continue
            n = self._missed.get(k, 0) + 1
            if n > grace:
                notes.append(f'{k}: unseen {n - 1} snapshots -> removed')
                continue
            missed[k] = n
            named[k] = copy.deepcopy(p)
            notes.append(f'{k}: not seen ({n}/{grace}) -> kept')
        self._missed = missed

    # ------------------------------------------------ service plumbing

    @staticmethod
    def _wait(fut, timeout):
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
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

    # ------------------------------------------------ debug / viz

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
            col = palette.get(o['label_source'], (0, 0, 255))
            cv2.rectangle(img, (u0, v0), (u1, v1), col, 2)
            cv2.putText(img, f"{oid} {o['confidence']:.2f}", (u0, max(v0 - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def publish_workspace_marker(self):
        x0, x1, y0, y1 = self.workspace
        xy = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        bot = [[x, y, self.table_z_at(x, y)] for x, y in xy]
        top = [[x, y, z + self.cfg['max_height']] for x, y, z in bot]
        c = bot + top
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.type, m.action = 'workspace', 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.006
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 0.8
        for a, b in edges:
            m.points += [Point(x=c[a][0], y=c[a][1], z=c[a][2]),
                         Point(x=c[b][0], y=c[b][1], z=c[b][2])]
        self.workspace_pub.publish(MarkerArray(markers=[m]))

    # ------------------------------------------------ main pipeline

    def run_snapshot(self, n, remove_missing=True):
        self._mask_uv = None
        report = {'dropped': [], 'notes': []}
        color, frames, depth, K, cam, K_hd, hd_frame = self.grab(n)
        T = self.tf_matrix(self.base_frame, cam)
        pts, uv = self.backproject(depth, K, T)

        if self.cfg['save_dir']:
            os.makedirs(self.cfg['save_dir'], exist_ok=True)
            np.savez_compressed(os.path.join(self.cfg['save_dir'], f'{int(time.time())}.npz'),
                                color=color, depth=depth, K=K, T=T)

        pts, uv = self.filter_workspace(pts, uv)
        pts, uv = self.denoise(pts, uv)
        self.get_logger().info(f'points in workspace above table: {len(pts)}')

        H = None
        if hd_frame is not None and self.plane is not None:
            H = self._hd_to_sd_homography(K_hd, self.tf_matrix(self.base_frame, hd_frame), K, T)
            if H is None:
                self.get_logger().warn('HD->SD homography failed; YOLO boxes may be misaligned')

        dets = self.detect(frames, H)
        if self.yolo is not None:
            self.get_logger().info('YOLO: ' + (', '.join(f'{d.label} {d.conf:.2f}' for d in dets)
                                               or 'no stable detections'))
        clusters = self.cluster(pts, uv)
        if dets and self.cfg['split_by_boxes']:
            clusters = self.split_merged(clusters, dets)

        matches, unmatched = self.associate(clusters, dets)
        matched_det = {j for _, j in matches}
        report['dropped'] = [f'{d.label} {d.conf:.2f} (no cluster under box)'
                             for j, d in enumerate(dets) if j not in matched_det]
        objs = [self._make_object(clusters[i], dets[j], color) for i, j in matches]
        objs += [self._make_object(clusters[i], None, color) for i in unmatched]

        prev, attached = self.fetch_scene()
        prev.pop('table_fit', None)
        self.stabilise_labels(objs, prev)
        named = self.assign_ids(objs, prev)
        for o in named.values():
            o['description'] = (f"{o['color']} {o['label']}"
                                + (' (unidentified)' if o['label_source'] == 'fallback' else ''))
        if remove_missing:
            self.carry_unseen(named, prev, attached, report['notes'])
        table = self._make_table_object()
        if table is not None:
            named['table_fit'] = table

        self.publish_debug(color, named, dets)
        for line in report['notes'] + report['dropped']:
            self.get_logger().info(f'  {line}')
        return named, report

    def snapshot_cb(self, req, res):
        n = int(req.num_frames) or int(self.cfg['num_frames'])
        try:
            named, report = self.run_snapshot(n, req.remove_missing)
            payload = {k: {kk: vv for kk, vv in o.items() if not kk.startswith('_')}
                       for k, o in named.items()}
            push_res, res.message = self.push(payload, req.remove_missing)
            res.success = bool(push_res and push_res.success)
            res.summary_json = json.dumps({
                'objects': {k: {'label': o['label'], 'source': o['label_source'],
                                'conf': o['confidence']} for k, o in payload.items()},
                'unidentified': [k for k, o in payload.items() if o['label_source'] == 'fallback'],
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()