import time
import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco(stream, mtx, dist, markerLength):
    # Create an ArUco dictionary and parameters for detector, ballsucker uses 7x7_1000
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters()

    # Capture frame-by-frame
    frame = stream.get_frame()

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)
    # Initialize lists to store marker corners and ids
    aruco_corners = []
    aruco_ids = []

    # Check if any marker detected
    if ids is not None:
        # Draw detected markers on the frame for visualization
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Store detected corners and ids
        aruco_corners = corners
        aruco_ids = ids.flatten()  # Flatten the ids array for easier handling
        
        # Print marker info every second, should be turned off, on for debug, made a function to turn it off ez
        print_marker_info(aruco_ids, aruco_corners)

    return aruco_corners, aruco_ids, frame

def print_marker_info(ids, corners, last_print_time=None, print_interval=1):
    current_time = time.time()
    if last_print_time is None or (current_time - last_print_time) >= print_interval:
        print(f"Marker locations at {time.ctime()} are:")
        for i, corner in zip(ids, corners):
            print(f"Marker {i}: Corners = {corner}")
        return current_time
    return last_print_time