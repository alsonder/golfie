import time
import cv2
import cv2.aruco as aruco
import numpy as np

# Inspired by https://docs.opencv.org/4.9.0/d5/dae/tutorial_aruco_detection.html

def detect_aruco(stream, mtx, dist, markerLength):
    # Create an ArUco dictionary and detector
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)

    # Set coordinate system
    objPoints = np.array([[-markerLength/2, markerLength/2, 0],
                          [markerLength/2, markerLength/2, 0],
                          [markerLength/2, -markerLength/2, 0],
                          [-markerLength/2, -markerLength/2, 0]], dtype=np.float32)

    last_print_time = time.time()

    while True:
        # Capture frame-by-frame
        frame = stream.get_frame()

        # Make a copy of the frame
        frame_copy = frame.copy()

        # Detect the markers in the image
        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=parameters)
        
        # Initialize lists to store positions and orientations
        position = []
        orientation = []

        # If at least one marker detected
        if ids is not None:
            # Draw the detected markers on the image
            aruco.drawDetectedMarkers(frame_copy, corners, ids)

            nMarkers = len(corners)
            rvecs, tvecs = [np.zeros((nMarkers, 3, 1), dtype=np.float32) for _ in range(2)]

            # Calculate pose for each marker
            for i in range(nMarkers):
                _, rvecs[i], tvecs[i] = cv2.solvePnP(objPoints, corners[i], mtx, dist)

                # Store position and orientation
                position.append(tvecs[i])
                orientation.append(rvecs[i])

            # Draw axis for each marker
            for i in range(len(ids)):
                try:
                    cv2.drawFrameAxes(frame_copy, mtx, dist, rvecs[i], tvecs[i], 0.1)
                except AttributeError:
                    print("drawFrameAxes function not available in this version of OpenCV")

            # Print location every second
            if time.time() - last_print_time >= 1:
                print(f"Marker locations at {time.ctime()} are:")
                for i in range(len(ids)):
                    print(f"Marker {ids[i]}: rvec = {rvecs[i]}, tvec = {tvecs[i]}")
                last_print_time = time.time()

        # Show resulting image and close window
        cv2.imshow('out', frame_copy)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    return position, orientation, frame