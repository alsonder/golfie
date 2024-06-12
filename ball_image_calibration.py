import cv2
import numpy as np
from livestream.livestream import LiveStream

def calibrate_and_detect_balls(stream, mtx=None, dist=None):
    def nothing(x):
        pass

    cv2.namedWindow('Adjustments')
    cv2.createTrackbar('Gaussian Blur', 'Adjustments', 0, 10, nothing)  # Range 0-10, ensures we can turn it off with 0
    
    # Hough Circle Transform parameters
    cv2.createTrackbar('Param1', 'Adjustments', 50, 100, nothing)
    cv2.createTrackbar('Param2', 'Adjustments', 30, 100, nothing)
    cv2.createTrackbar('Min Radius', 'Adjustments', 0, 100, nothing)
    cv2.createTrackbar('Max Radius', 'Adjustments', 50, 100, nothing)

    # Initialize CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    while True:
        frame = stream.get_frame()

        if mtx is not None and dist is not None:
            frame = cv2.undistort(frame, mtx, dist)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply the CLAHE filter to the grayscale image
        clahe_gray = clahe.apply(gray)

        # Apply Gaussian Blur based on the trackbar position
        gaussian_blur = cv2.getTrackbarPos('Gaussian Blur', 'Adjustments')
        if gaussian_blur > 0:
            # Ensuring the kernel size is odd
            gaussian_blur_size = gaussian_blur * 2 + 1
            blurred_gray = cv2.GaussianBlur(clahe_gray, (gaussian_blur_size, gaussian_blur_size), 0)
        else:
            blurred_gray = clahe_gray

        # Hough Circles detection on the processed image
        param1 = cv2.getTrackbarPos('Param1', 'Adjustments')
        param2 = cv2.getTrackbarPos('Param2', 'Adjustments')
        minRadius = cv2.getTrackbarPos('Min Radius', 'Adjustments')
        maxRadius = cv2.getTrackbarPos('Max Radius', 'Adjustments')
        
        circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, 1, 20, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        cv2.imshow('Calibration View', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



