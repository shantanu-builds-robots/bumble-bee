import cv2
import numpy as np

def generate_aruco_marker(marker_id, marker_size, dictionary, save_path):
    """
    Generate an ArUco marker image and save it.

    Parameters:
    - marker_id: ID of the marker to be generated.
    - marker_size: Size of the marker image in pixels.
    - dictionary: The dictionary of markers to use (e.g., DICT_4X4_50).
    - save_path: Path where the marker image will be saved.
    """
    # Create the specified dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
    # Generate the marker image
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)
    
    # Save the marker image
    cv2.imwrite(save_path, marker_image)
    print(f"Marker {marker_id} saved at {save_path}")

# Example usage
if __name__ == "__main__":
    # Parameters
    marker_id = 23                    # ID of the marker
    marker_size = 200                 # Size of the marker in pixels
    dictionary = cv2.aruco.DICT_4X4_50 # Dictionary type
    save_path = "aruco_marker_23.png" # Save path

    # Generate and save the marker
    generate_aruco_marker(marker_id, marker_size, dictionary, save_path)
