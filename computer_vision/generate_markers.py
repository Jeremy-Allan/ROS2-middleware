import cv2
import numpy as np
import os

def generate_aruco_markers():
    # Define the dictionary of ArUco markers (6x6 grid, 250 total unique markers)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Specify which IDs you want to generate:
    # ID 0 -> "apple", ID 1 -> "banana", ID 2 -> "tray"
    # ID 10-13 -> Table Anchors
    marker_ids = [0, 1, 2, 10, 11, 12, 13]

    # Set the size of the output images in pixels (high resolution for clean printing)
    marker_size_px = 400

    # Ensure output directory exists
    output_dir = os.path.dirname(os.path.abspath(__file__))

    print(f"Generating ArUco markers in: {output_dir}")

    for marker_id in marker_ids:
        # Create an empty image for the marker
        marker_img = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)
        
        # Generate the marker
        # In newer OpenCV versions (>= 4.7.0), use cv2.aruco.generateImageMarker.
        # Fallback to cv2.aruco.drawMarker if on an older version.
        try:
            cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px, marker_img, 1)
        except AttributeError:
            # Fallback for older OpenCV versions
            marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px, 1)

        # Save the marker as PNG
        filename = os.path.join(output_dir, f"aruco_marker_{marker_id}.png")
        cv2.imwrite(filename, marker_img)
        print(f" -> Generated {os.path.basename(filename)} (ID: {marker_id})")

    print("\nNext Steps:")
    print("1. Print these images out on standard paper.")
    print("2. Ensure they are not scaled to fit page; print them at a fixed size (e.g. 4cm x 4cm or 5cm x 5cm).")
    print("3. Measure the final printed outer black square with a ruler and convert it to meters (e.g., 4cm = 0.04m).")

if __name__ == "__main__":
    generate_aruco_markers()
