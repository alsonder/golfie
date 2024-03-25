import cv2
import numpy as np
from livestream.livestream import LiveStream

"""This File is made to run as a calibration after the cameracalibration with the checkerboard, it is sometimes necessary to run more t
times. Adjustment guide:

    1: make sure that its using gray scale, it yielded the best results for detecting circles. look in ball_detection.py for latest settings
    2: set hsv values, (you dont need to adjust the uppper usually, just the lower H and lower s is usually fine)
    3: circle parameters: the most important settings here. These should follow whats in the ball_detection.py file atm, these
    are the last known parameters. I found that a param2 setting of 13-16 is best, but this is in the limit of not detecting the balls, 
    try to set the radius first, depending on the camera angle the angle should be adjusted, find the minimum radius first, find
    the value it doesnt detect a ball, or the ball the furthest away from the camera, then set it one less, same for max radius, this 
    filters away most false positives.
    The goal is to have them flickering at the highest frequency, so balance frequency blinking, and not detecting any false positives
    the moving average class will handle it if they blink but see if the frequency can be adjusted so it blinks fast."""

def calibrate_and_detect_balls(stream, mtx=None, dist=None):
    def nothing(x):
        pass

    # Create trackbars for adjustment
    cv2.namedWindow('HSV Adjustments')
    # Initialize HSV trackbars
    cv2.createTrackbar('Lower H', 'HSV Adjustments', 0, 179, nothing)
    cv2.createTrackbar('Lower S', 'HSV Adjustments', 0, 255, nothing)
    cv2.createTrackbar('Lower V', 'HSV Adjustments', 0, 255, nothing)
    cv2.createTrackbar('Upper H', 'HSV Adjustments', 179, 179, nothing)
    cv2.createTrackbar('Upper S', 'HSV Adjustments', 255, 255, nothing)
    cv2.createTrackbar('Upper V', 'HSV Adjustments', 255, 255, nothing)
    cv2.createTrackbar('Gaussian Blur', 'HSV Adjustments', 0, 10, nothing)
    cv2.createTrackbar('Dilate', 'HSV Adjustments', 0, 10, nothing)

    # Initialize Hough Circle Transform trackbars
    cv2.namedWindow('Hough Circles Adjustments')
    cv2.createTrackbar('Param1', 'Hough Circles Adjustments', 50, 100, nothing)
    cv2.createTrackbar('Param2', 'Hough Circles Adjustments', 30, 100, nothing)
    cv2.createTrackbar('Min Radius', 'Hough Circles Adjustments', 0, 100, nothing)
    cv2.createTrackbar('Max Radius', 'Hough Circles Adjustments', 50, 100, nothing)

    while True:
        frame = stream.get_frame() 
        
        if mtx is not None and dist is not None:
            frame = cv2.undistort(frame, mtx, dist)
        
        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale which we currently use, due to better results in detection
        
        # Get current positions of trackbars
        lh = cv2.getTrackbarPos('Lower H', 'HSV Adjustments')
        ls = cv2.getTrackbarPos('Lower S', 'HSV Adjustments')
        lv = cv2.getTrackbarPos('Lower V', 'HSV Adjustments')
        uh = cv2.getTrackbarPos('Upper H', 'HSV Adjustments')
        us = cv2.getTrackbarPos('Upper S', 'HSV Adjustments')
        uv = cv2.getTrackbarPos('Upper V', 'HSV Adjustments')
        
        cv2.createTrackbar('Adaptive Method', 'HSV Adjustments', 0, 1, nothing) 
        cv2.createTrackbar('Block Size', 'HSV Adjustments', 3, 100, nothing)
        cv2.createTrackbar('Constant C', 'HSV Adjustments', 0, 50, nothing) 
        gaussian_blur = cv2.getTrackbarPos('Gaussian Blur', 'HSV Adjustments') * 2 + 1  # Ensuring odd number
        dilate_iter = cv2.getTrackbarPos('Dilate', 'HSV Adjustments')
        adaptive_method = cv2.getTrackbarPos('Adaptive Method', 'HSV Adjustments')
        block_size = cv2.getTrackbarPos('Block Size', 'HSV Adjustments') * 2 + 3 # Ensure odd block size
        c = cv2.getTrackbarPos('Constant C', 'HSV Adjustments')

        # Define HSV range and create a mask
        lower_hsv = np.array([lh, ls, lv])
        upper_hsv = np.array([uh, us, uv])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Apply adaptive thresholding to grayscale image (sometimes its good, sometimes its bad, adjust slider to see)
        if adaptive_method == 0:
            method = cv2.ADAPTIVE_THRESH_MEAN_C
        else:
            method = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        mask = cv2.adaptiveThreshold(gray, 255, method, cv2.THRESH_BINARY, block_size, c)

        # Apply Gaussian Blur to the mask (this is sometimes needed sometimes not, adjjust blur slider to see the result, but adjust circle params first.)
        if gaussian_blur > 0:
            mask = cv2.GaussianBlur(gray, (gaussian_blur, gaussian_blur), 0)

        # Dilate mask (never used, but it could be beneficial if there is alot of reflection, but will mess with the circle nodes in the hough algo)
        if dilate_iter > 0:
            mask = cv2.dilate(mask, None, iterations=dilate_iter)

        # Hough Circles Detection parameters, this is the main adjustment, if using grayscale over the mask
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=cv2.getTrackbarPos('Param1', 'Hough Circles Adjustments'),
                                   param2=cv2.getTrackbarPos('Param2', 'Hough Circles Adjustments'),
                                   minRadius=cv2.getTrackbarPos('Min Radius', 'Hough Circles Adjustments'),
                                   maxRadius=cv2.getTrackbarPos('Max Radius', 'Hough Circles Adjustments'))

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        cv2.imshow('adjusted frame', frame)
        cv2.imshow('mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
