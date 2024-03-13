import numpy as np
import cv2
import time
from livestream import livestream  # Corrected import statement

def calibrate_camera(stream):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    cap = stream

    start_time = time.time()
    while time.time() - start_time < 30:  # calibrate for 30 seconds
        frame = cap.get_frame()  # use get_frame instead of read
        if frame is None:  # check if the frame is None instead of using ret
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(frame, (9,6), corners2, ret)
            cv2.imshow('img', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #cv2.destroyAllWindows()
    #cap.release()

    if len(objpoints) > 0 and len(imgpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return ret, mtx, dist, rvecs, tvecs
    else:
        return False, None, None, None, None