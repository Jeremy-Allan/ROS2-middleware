#!/usr/bin/env python3
"""Vision snapshot node — Kinect v2 -> object_dictionary via environment_mapping_node.

Snapshot flow:
  1. Capture N depth frames, take per-pixel median (kills ToF noise + flying pixels).
  2. Backproject to base_link using camera intrinsics + TF.
  3. Filter to workspace above table_z.
  4. DBSCAN cluster.
  5. Fit oriented bounding box (outlier-trimmed, minAreaRect on XY).
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
from sensor_msgs.msg import CameraInfo, Image
from sklearn.cluster import DBSCAN
from tf2_ros import Buffer, TransformListener

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
    center: np.ndarray  # x, y, table_z + h/2
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
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('min_height', 0.008)
        self.declare_parameter('max_height', 0.30)
        self.declare_parameter('cluster_eps', 0.02)
        self.declare_parameter('cluster_min_points', 30)
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('save_dir', '')

        self.base_frame = self.get_parameter('base_frame').value
        self.workspace = self.get_parameter('workspace').value
        self.table_z   = self.get_parameter('table_z').value

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

    def _on_color(self, msg):
        self._color = msg

    def _on_info(self, msg):
        self._info = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depths.append(msg)

    def _depth_to_m(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
        if msg.encoding in ('16UC1', 'mono16'):
            d /= 1000.0
        d[d == 0] = np.nan
        return d

    # ------------------------------------------------ capture

    def grab(self, n):
        """Collect n fresh depth frames -> per-pixel median. Returns (bgr, depth, K, cam_frame)."""
        with self._lock:
            self._depths.clear()
        t0 = time.time()
        while len(self._depths) < n:
            if time.time() - t0 > 10.0 + 0.3 * n or not rclpy.ok():
                raise TimeoutError('Timed out waiting for depth frames')
            time.sleep(0.02)
        with self._lock:
            msgs = list(self._depths)[-n:]

        if self._color is None or self._info is None:
            raise RuntimeError('No colour image / camera_info yet')

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

    # ------------------------------------------------ geometry

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
        x0, x1, y0, y1 = self.workspace
        h = pts[:, 2] - self.table_z
        keep = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1)
                & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
                & (h >= self.get_parameter('min_height').value)
                & (h <= self.get_parameter('max_height').value))
        return pts[keep], uv[keep]

    def build_cluster(self, pts, uv):
        """Fit an oriented box. Trims the outer 2% of points along x and y
        before minAreaRect, which kills the flying-pixel outliers that
        otherwise inflate the box by several cm on each side."""
        # --- outlier trim ---
        xlo, xhi = np.percentile(pts[:, 0], [2, 98])
        ylo, yhi = np.percentile(pts[:, 1], [2, 98])
        m = ((pts[:, 0] >= xlo) & (pts[:, 0] <= xhi)
             & (pts[:, 1] >= ylo) & (pts[:, 1] <= yhi))
        pts, uv = pts[m], uv[m]
        # --------------------

        xy = pts[:, :2].astype(np.float32)
        (cx, cy), (w, d), ang = cv2.minAreaRect(xy)

        # 95th percentile for height (was 98) — ignore top-of-object outliers
        top = float(np.percentile(pts[:, 2], 95))
        h = max(top - self.table_z, 0.01)

        u0, v0 = uv.min(axis=0)
        u1, v1 = uv.max(axis=0)
        return Cluster(
            pts=pts, uv=uv,
            bbox2d=(float(u0), float(v0), float(u1), float(v1)),
            center=np.array([cx, cy, self.table_z + h / 2.0]),
            dims=np.array([max(w, 0.01), max(d, 0.01), h]),
            yaw=math.radians(ang),
        )

    def cluster(self, pts, uv):
        n_min = self.get_parameter('cluster_min_points').value
        if len(pts) < n_min:
            return []
        labels = DBSCAN(eps=self.get_parameter('cluster_eps').value,
                        min_samples=10).fit_predict(pts)
        out = [self.build_cluster(pts[labels == l], uv[labels == l])
               for l in sorted(set(labels) - {-1})
               if (labels == l).sum() >= n_min]
        return sorted(out, key=lambda c: (c.center[1], c.center[0]))

    # ------------------------------------------------ YOLO

    def _detect(self, color_bgr):
        """Return a list of Detection. Empty when yolo_model param is ''."""
        if self.yolo is None:
            return []
        results = self.yolo.predict(
            color_bgr,
            conf=self.get_parameter('yolo_conf').value,
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
                             'z': float(self.table_z + h / 2.0)},
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
        color, depth, K, cam_frame = self.grab(n)
        T = self.tf_matrix(self.base_frame, cam_frame)
        pts, uv = self.backproject(depth, K, T)

        self.get_logger().info(f'raw points after backproject: {len(pts)}')
        if len(pts) > 0:
            self.get_logger().info(
                f'  bounds x[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}] '
                f'y[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}] '
                f'z[{pts[:,2].min():.3f}, {pts[:,2].max():.3f}]')

            # z-histogram for finding table_z: biggest peak is the table surface
            hist, edges = np.histogram(pts[:, 2], bins=40)
            top5 = sorted(zip(hist, edges[:-1]), reverse=True)[:5]
            self.get_logger().info(
                '  z peaks: ' + ', '.join(f'({c}, {z:.3f})' for c, z in top5))

        # optional: save raw frames for offline replay / tuning
        save_dir = self.get_parameter('save_dir').value
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            path = os.path.join(save_dir, f'{int(time.time())}.npz')
            np.savez_compressed(path, color=color, depth=depth, K=K, T=T)
            self.get_logger().info(f'saved snapshot: {path}')

        pts, uv = self.filter_workspace(pts, uv)
        self.get_logger().info(f'after workspace filter: {len(pts)}')

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

        # YOLO-only detections (no cluster): dropped for now. When YOLO is on,
        # add a class-prior fallback here (see earlier discussion) if you want
        # dark/shiny objects that the ToF can't see.
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