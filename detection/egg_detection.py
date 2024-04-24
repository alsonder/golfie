import cv2
import numpy as np

"""This function detects all circles using houghcircles, it outputs the coordinates in pixels and sends it to the confirm balls, 
    for insertion in the list, this also detects loose balls in the wild, if there is a ball being pushed, it notices
    and sends it to confirmation"""


def detect_egg(frame, mtx, dist):
    frame_undistorted = cv2.undistort(frame, mtx, dist)
    hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    # Apply the CLAHE filter to the grayscale image
    clahe_gray = clahe.apply(gray)
    # Change below according to the best setting found during calibrate_and_detect_balls()
    lower_hsv = np.array([0, 0, 0])
    upper_hsv = np.array([179, 255, 255])
    gaussian_blur = 6
    param1 = 50
    param2 = 12
    min_radius = 13
    max_radius = 17

    # apply mask which circle detection operates
    blurred_gray = cv2.GaussianBlur(clahe_gray, (gaussian_blur, gaussian_blur), 0)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.dilate(mask, None, iterations=1)

    circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, 1, 1,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)
    ball_list = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            ball_list.append([i[0], i[1]])


    return ball_list  # Returns only the list of detected balls without confirmation status (list of unfirm balls)



