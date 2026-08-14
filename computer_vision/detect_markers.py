import cv2
import numpy as np
import os

# Define the target names corresponding to our printed IDs
TARGET_NAMES = {
    0: "apple",
    1: "banana",
    2: "tray"
}

# The physical size of your printed outer square in meters (7cm = 0.07m)
MARKER_SIZE = 0.07

def main():
    # 1. Connect to video capture (Iriun Webcam is usually index 0, 1, or 2)
    # If index 0 opens your built-in mac camera, try changing it to 1 or 2.
    camera_index = 0
    print(f"Opening camera index {camera_index}...")
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"Error: Could not open camera on index {camera_index}.")
        print("Please check your Iriun connection and try indices 1 or 2 if needed.")
        return

    # Set camera resolution to a standard size
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Let the camera warm up to read correct dimensions
    ret, test_frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from camera.")
        return
    
    height, width = test_frame.shape[:2]
    print(f"Camera opened successfully! Resolution: {width}x{height}")

    # 2. Setup Camera Intrinsics Heuristics (Fallback if no custom calibration)
    # Standard pinhole approximation: Focal length ~ max dimension of image
    focal_length = max(width, height)
    center_x = width / 2
    center_y = height / 2
    
    # Intrinsic Matrix
    camera_matrix = np.array([
        [focal_length, 0, center_x],
        [0, focal_length, center_y],
        [0, 0, 1]
    ], dtype=np.float32)
    
    # Distortion Coefficients (Assume zero lens distortion initially)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    # Define the 3D coordinates of the physical marker's 4 corners relative to its center.
    # Used for solvePnP pose estimation.
    half_size = MARKER_SIZE / 2.0
    obj_points = np.array([
        [-half_size,  half_size, 0],  # Top-Left
        [ half_size,  half_size, 0],  # Top-Right
        [ half_size, -half_size, 0],  # Bottom-Right
        [-half_size, -half_size, 0]   # Bottom-Left
    ], dtype=np.float32)

    # 3. Setup ArUco Detector dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    print("\nStarting live tracking loop. Press 'q' to quit...")
    print("Keep your physical tags in the camera's field of view.")
    print("---------------------------------------------------------")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers (Handles old and new OpenCV APIs)
        try:
            # Modern OpenCV (>= 4.7.0)
            detector_params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        except AttributeError:
            # Older OpenCV (< 4.7.0)
            detector_params = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

        # If any markers were detected
        if ids is not None:
            # Flatten the ids array
            ids_flat = ids.flatten()
            
            # Draw boundaries around detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for idx, marker_id in enumerate(ids_flat):
                # Retrieve the 2D corners for this specific marker
                marker_corners = corners[idx][0]

                # 4. Estimate 3D Pose (rvec = rotation vector, tvec = translation vector)
                # SolvePnP calculates exact 3D distance relative to the camera lens.
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success:
                    # tvec represents the translation [X, Y, Z] relative to the camera lens
                    # x_cam: horizontal distance (left is negative, right is positive)
                    # y_cam: vertical distance (up is negative, down is positive)
                    # z_cam: direct depth distance away from the camera lens
                    x_cam, y_cam, z_cam = tvec.flatten()

                    # Convert to centimeters for readable print
                    x_cm = x_cam * 100
                    y_cm = y_cam * 100
                    z_cm = z_cam * 100

                    # Map ID to friendly name
                    name = TARGET_NAMES.get(marker_id, f"unknown_tag_{marker_id}")

                    # Print out the live coordinate
                    print(f"ID {marker_id} ({name:6s}) -> Coordinates in Camera Frame: X={x_cm:6.1f}cm, Y={y_cm:6.1f}cm, Z={z_cm:6.1f}cm", end="\r", flush=True)

                    # Draw 3D axis on the marker to visualize orientation (Length = 5cm)
                    try:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    except AttributeError:
                        # Fallback for older OpenCV versions
                        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                    # Overlay coordinate text on image
                    text_pos = (int(marker_corners[0][0]), int(marker_corners[0][1]) - 15)
                    label = f"{name} X:{x_cm:.1f} Y:{y_cm:.1f} Z:{z_cm:.1f}cm"
                    cv2.putText(frame, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the video feed with overlays
        cv2.imshow("Live ArUco 3D Tracking", frame)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Clean up
    print("\nShutting down tracking loop.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
