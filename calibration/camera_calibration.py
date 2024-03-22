import cv2
import numpy as np
from livestream import livestream

# heavyly inspired by https://www.ostirion.net/post/webcam-calibration-with-opencv-directly-from-the-stream
# AKSS: I have made some changes to the original code to make it work with the livestream class
# and added some comments to make it easier to understand

CHECKERBOARD = (9, 6) # Size of the checkerboard
MIN_POINTS = 50

def calibrate_camera(stream):
    """
    Calibrates the camera using a checkerboard pattern. 9*6 openCV chessboard (ask Aleksander)

    This function captures frames from the provided stream, detects the checkerboard pattern in each frame, and uses the detected points to calibrate the camera. The calibration parameters are then returned.

    Parameters:
    stream (livestream.LiveStream): The livestream object from which frames are captured.

    Returns:
    tuple: A tuple containing the following calibration parameters:
        - ret (bool): The result of the calibration process. True if successful, False otherwise.
        - mtx (numpy.ndarray): The camera matrix.
        - dist (numpy.ndarray): The distortion coefficients.
        - rvecs (list): The rotation vectors.
        - tvecs (list): The translation vectors.
    """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    threedpoints = []
    twodpoints = []

    # Start capturing images for calibration
    cap = stream

    # Loop over the frames
    while True:
        # Grab a frame
        frame = cap.get_frame()
        if frame is None:
            break
        
        # convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # finding the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret == True:
            threedpoints.append(objectp3d)

            corners2 = cv2.cornerSubPix(gray, corners, CHECKERBOARD, (-1, -1), criteria)
            twodpoints.append(corners2)

            # Draw and display the corners
            if len(twodpoints) > MIN_POINTS:
                break
            
            # Draw and display the corners
            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

        cv2.imshow('img', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # If we have enough points to calibrate    
    if len(threedpoints) > 0 and len(twodpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(threedpoints, twodpoints, gray.shape[::-1], None, None)
        
        #print("Camera matrix:")
        #print(mtx)

        print("\nDistortion coefficient from calibration module:")
        print(dist)

        #print("\nRotation Vectors:")
        #print(rvecs)

        #print("\nTranslation Vectors:")
        #print(tvecs)

        #return the calibration parameters
        return ret, mtx, dist, rvecs, tvecs
    else:
        return False, None, None, None, None