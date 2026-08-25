import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from kinova_interfaces.srv import UpdateObjectPose
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

# Map ArUco IDs to object names defined in object_dictionary.json
TARGET_NAMES = {
    0: "red_cube",
    1: "blue_cube",
    2: "delivery_tray"
}

MARKER_SIZE = 0.07  # meters

class CVDetectionNode(Node):
    def __init__(self):
        super().__init__('cv_detection_node')
        
        # Parameters
        self.declare_parameter('camera_source', '0')
        self.declare_parameter('use_markers', True)
        
        source = self.get_parameter('camera_source').value
        
        # Determine if source is an index or a URL
        try:
            # If it's a digit (like "0"), treat it as a camera index
            camera_index = int(source)
            self.get_logger().info(f"Opening local camera index {camera_index}...")
            self.cap = cv2.VideoCapture(camera_index)
        except ValueError:
            # If it's a string (like a URL), treat it as a stream source
            self.get_logger().info(f"Opening IP camera stream: {source}...")
            self.cap = cv2.VideoCapture(source)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera source: {source}")
        
        # Camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Intrinsics Heuristics (as per POC)
        # In a real setup, these should be loaded from a calibration file
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
        
        # Detection Timer (10Hz)
        self.timer = self.create_timer(0.1, self.detection_callback)
        self.get_logger().info("CV Detection Node initialized")

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
                    # 1. Create Pose in Camera Frame
                    pose_camera = Pose()
                    tvec = tvec.flatten()
                    pose_camera.position.x = float(tvec[0])
                    pose_camera.position.y = float(tvec[1])
                    pose_camera.position.z = float(tvec[2])
                    
                    # Convert rotation vector to quaternion
                    rmat, _ = cv2.Rodrigues(rvec)
                    quat = R.from_matrix(rmat).as_quat() # [x, y, z, w]
                    pose_camera.orientation.x = quat[0]
                    pose_camera.orientation.y = quat[1]
                    pose_camera.orientation.z = quat[2]
                    pose_camera.orientation.w = quat[3]
                    
                    # 2. Transform to Robot Frame (base_link)
                    self.transform_and_update(obj_name, pose_camera)

    def transform_and_update(self, obj_name, pose_camera):
        try:
            # Look up transform from base_link to camera_link
            # We assume the camera frame in OpenCV matches our URDF camera_link (Z forward, X right, Y down)
            # Or we might need an intermediate 'camera_color_optical_frame' style transform
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            
            # Use tf2_geometry_msgs to transform the pose
            pose_robot = tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
            
            # 3. Update the environment_mapping_node
            self.call_update_service(obj_name, pose_robot)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform {obj_name} to base_link: {str(e)}", throttle_duration_sec=5.0)

    def call_update_service(self, obj_name, pose):
        if not self.update_client.wait_for_service(timeout_sec=1.0):
            return
        
        req = UpdateObjectPose.Request()
        req.object_id = obj_name
        req.pose = pose
        
        future = self.update_client.call_async(req)
        # We don't wait for the result here to keep the loop fast

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
