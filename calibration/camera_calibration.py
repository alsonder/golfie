import cv2
import numpy as np
from livestream import livestream

CHECKERBOARD = (9, 6)
MIN_POINTS = 50

def calibrate_camera(stream):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    threedpoints = []
    twodpoints = []

    cap = stream

    while True:
        frame = cap.get_frame()
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            threedpoints.append(objectp3d)

            corners2 = cv2.cornerSubPix(gray, corners, CHECKERBOARD, (-1, -1), criteria)
            twodpoints.append(corners2)

            if len(twodpoints) > MIN_POINTS:
                break

            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

        cv2.imshow('img', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if len(threedpoints) > 0 and len(twodpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(threedpoints, twodpoints, gray.shape[::-1], None, None)
        
        print("Camera matrix:")
        print(mtx)

        print("\nDistortion coefficient:")
        print(dist)

        print("\nRotation Vectors:")
        print(rvecs)

        print("\nTranslation Vectors:")
        print(tvecs)

        return ret, mtx, dist, rvecs, tvecs
    else:
        return False, None, None, None, None