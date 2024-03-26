import cv2
import numpy as np

"""This function detects all balls using houghcircles, it outputs the coordinates in pixels and sends it to the confirm balls, 
    for insertion in the list, this also detects loose balls in the wild, if there is a ball being pushed, it notices
    and sends it to confirmation"""

def detect_balls(frame, mtx, dist):
    frame_undistorted = cv2.undistort(frame, mtx, dist)
    hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    
    # Change below according to the best setting found during calibrate_and_detect_balls()
    lower_hsv = np.array([160, 215, 0])
    upper_hsv = np.array([179, 255, 255])
    gaussian_blur = 1
    param1 = 57
    param2 = 13
    min_radius = 4
    max_radius = 6

    # apply mask which circle detection operates
    blurred_gray = cv2.GaussianBlur(gray, (gaussian_blur, gaussian_blur), 0)
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
