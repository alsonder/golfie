import cv2
import numpy as np
import pathfinding

"""This function will detect a circle on the thick part of the egg. Then calculate points on a circle periphery,
that encapsulates the egg.The standard amount of points made is 360, because it increments 1 degree / point.
The function then returns a numpy array with touples containing coordinates. Example: [(150,300),(145,315)]"""


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
    gaussian_blur = 1
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
    egg_list = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            egg_list.append([i[0], i[1]])

    egg_circle = []
    for ball in egg_list:
        for i in range(360):
            X = ball[0] + (20 * np.cos(360/(i+1)))  
            Y = ball[1] + (20 * np.sin(360/(i+1)))
            egg_circle.append([X,Y])

    return egg_circle  # Returns the 360points created just above




