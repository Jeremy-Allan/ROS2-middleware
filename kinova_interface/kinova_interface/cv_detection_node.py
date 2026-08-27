import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from kinova_interfaces.srv import UpdateObjectPose
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
import time

# Map ArUco IDs to object names defined in object_dictionary.json
TARGET_NAMES = {
    0: "red_cube",
    1: "blue_cube",
    2: "delivery_tray"
}

# Anchor Tag Definitions (IDs and their fixed coordinates in base_link)
# Table: 95cm (across front = Y) x 90cm (along side = X)
# Assuming robot base is at the geometric center (0,0)
ANCHOR_CONFIG = {
    10: {'x': 0.45,  'y': 0.475,  'z': 0.0, 'name': 'anchor_fl'},
    11: {'x': 0.45,  'y': -0.475, 'z': 0.0, 'name': 'anchor_fr'},
    12: {'x': -0.45, 'y': -0.475, 'z': 0.0, 'name': 'anchor_br'},
    13: {'x': -0.45, 'y': 0.475,  'z': 0.0, 'name': 'anchor_bl'}
}

MARKER_SIZE = 0.05  # meters

class CVDetectionNode(Node):
    def __init__(self):
        super().__init__('cv_detection_node')
        
        # Parameters
        self.declare_parameter('camera_source', '0')
        self.declare_parameter('use_markers', True)
        self.declare_parameter('dynamic_camera', True) # Enable dynamic camera localization
        
        source = self.get_parameter('camera_source').value
        self.dynamic_camera = self.get_parameter('dynamic_camera').value
        
        # Determine if source is an index or a URL
        try:
            camera_index = int(source)
            self.get_logger().info(f"Opening local camera index {camera_index}...")
            self.cap = cv2.VideoCapture(camera_index)
        except ValueError:
            self.get_logger().info(f"Opening IP camera stream: {source}...")
            self.cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera source: {source}")
        
        # Camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Intrinsics Heuristics
        width, height = 1280, 720
        focal_length = max(width, height)
        center_x = width / 2
        center_y = height / 2
        
        self.camera_matrix = np.array([
            [focal_length, 0, center_x],
            [0, focal_length, center_y],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        
        # ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.modern_opencv = True
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.modern_opencv = False

        half_size = MARKER_SIZE / 2.0
        self.obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # ROS2 Communication
        self.update_client = self.create_client(UpdateObjectPose, '/update_object_pose')
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Detection Timer (10Hz)
        self.timer = self.create_timer(0.1, self.detection_callback)
        self.get_logger().info("CV Detection Node initialized (Dynamic Camera: {})".format(self.dynamic_camera))

    def detection_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.modern_opencv:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        if ids is not None:
            ids_flat = ids.flatten()
            
            # --- 1. Dynamic Camera Self-Localization (Anchors) ---
            if self.dynamic_camera:
                self.localize_camera(corners, ids_flat)

            # --- 2. Object Detection & Update ---
            for idx, marker_id in enumerate(ids_flat):
                if marker_id not in TARGET_NAMES:
                    continue
                
                obj_name = TARGET_NAMES[marker_id]
                marker_corners = corners[idx][0]
                
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    marker_corners,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if success:
                    pose_camera = Pose()
                    tvec = tvec.flatten()
                    pose_camera.position.x = float(tvec[0])
                    pose_camera.position.y = float(tvec[1])
                    pose_camera.position.z = float(tvec[2])
                    
                    rmat, _ = cv2.Rodrigues(rvec)
                    quat = R.from_matrix(rmat).as_quat()
                    pose_camera.orientation.x = quat[0]
                    pose_camera.orientation.y = quat[1]
                    pose_camera.orientation.z = quat[2]
                    pose_camera.orientation.w = quat[3]
                    
                    self.transform_and_update(obj_name, pose_camera)

    def localize_camera(self, corners, ids_flat):
        """Calculates and broadcasts camera pose relative to base_link using anchor tags."""
        valid_anchors = []
        for idx, marker_id in enumerate(ids_flat):
            if marker_id in ANCHOR_CONFIG:
                valid_anchors.append((idx, marker_id))
        
        if not valid_anchors:
            return

        # Use the first visible anchor for localization (could be improved by averaging)
        idx, marker_id = valid_anchors[0]
        anchor_data = ANCHOR_CONFIG[marker_id]
        marker_corners = corners[idx][0]

        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if success:
            # tvec/rvec is Tag relative to Camera. We need Camera relative to Tag.
            rmat, _ = cv2.Rodrigues(rvec)
            
            # T_cam_tag = [rmat, tvec]
            # T_tag_cam = inverse(T_cam_tag)
            rmat_inv = rmat.T
            tvec_inv = -rmat_inv @ tvec
            
            # Now we have Camera position in Tag frame.
            # Convert to base_link frame: Camera_in_base = T_base_tag * Camera_in_tag
            # T_base_tag is just the anchor's fixed position.
            
            # Simple translation + rotation for now (assuming tag is aligned with base_link axes)
            # In a real setup, you'd use the full 4x4 matrix multiplication.
            cam_x = anchor_data['x'] + tvec_inv[0]
            cam_y = anchor_data['y'] + tvec_inv[1]
            cam_z = anchor_data['z'] + tvec_inv[2]

            # Broadcast the transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'camera_link'
            
            t.transform.translation.x = float(cam_x)
            t.transform.translation.y = float(cam_y)
            t.transform.translation.z = float(cam_z)
            
            # Camera orientation (needs careful handling of OpenCV vs ROS coordinate systems)
            # For this POC, we'll keep the orientation identity or basic inversion
            quat = R.from_matrix(rmat_inv).as_quat()
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(t)

    def transform_and_update(self, obj_name, pose_camera):
        try:
            # If dynamic, we wait for the transform we just broadcasted
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            pose_robot = tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
            self.call_update_service(obj_name, pose_robot)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}", throttle_duration_sec=5.0)

    def call_update_service(self, obj_name, pose):
        if not self.update_client.wait_for_service(timeout_sec=0.1):
            return
        
        req = UpdateObjectPose.Request()
        req.object_id = obj_name
        req.pose = pose
        self.update_client.call_async(req)

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CVDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
