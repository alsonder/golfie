import cv2 as cv

def detect_aruco(frame, calibration_params):
    # Create an ArUco dictionary and detector
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # Get the calibration parameters
    ret, mtx, dist, rvecs, tvecs = calibration_params

    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

    if markerIds is not None:
        # Draw the detected markers on the image
        frame = cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    return frame