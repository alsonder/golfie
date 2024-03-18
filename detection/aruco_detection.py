import cv2
import cv2.aruco as aruco

def detect_aruco(stream, calibration_params):
    # Create an ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # Get the calibration parameters
    ret, mtx, dist, rvecs, tvecs = calibration_params

    while True:
        # Grab a frame
        frame = stream.get_frame()
        if frame is None:
            break

        # Detect the markers in the image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters, cameraMatrix=mtx, distCoeff=dist)

        if ids is not None:
            # Draw the detected markers on the image
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    return frame

