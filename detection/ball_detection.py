import cv2
import numpy as np

def detect_balls(frame, mtx, dist):
    frame_undistorted = cv2.undistort(frame, mtx, dist)
    hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur for general ball detection
    gaussian_blur = 1
    blurred_gray = cv2.GaussianBlur(gray, (gaussian_blur, gaussian_blur), 0)

    # General HSV range for initial filtering
    lower_hsv = np.array([160, 215, 0])
    upper_hsv = np.array([179, 255, 255])

    # Create a mask based on general HSV range
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    masked_gray = cv2.bitwise_and(blurred_gray, blurred_gray, mask=mask)

    # Parameters for circle detection
    param1 = 57
    param2 = 13
    min_radius = 4
    max_radius = 6

    # Detect all potential balls using HoughCircles on the masked gray image
    circles = cv2.HoughCircles(masked_gray, cv2.HOUGH_GRADIENT, 1, 1,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)
    ball_list = []
    orange_ball = None

    # Expanded orange color range in HSV
    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([30, 255, 255])

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Retrieve original HSV values from unmasked image
            ball_hsv_value = hsv[i[1], i[0]]
            ball_list.append([i[0], i[1]])
            # Check if the detected ball is orange
            if lower_orange[0] <= ball_hsv_value[0] <= upper_orange[0] and \
               lower_orange[1] <= ball_hsv_value[1] <= upper_orange[1] and \
               lower_orange[2] <= ball_hsv_value[2] <= upper_orange[2]:
                orange_ball = [i[0], i[1]]  # Store the orange ball position

    # Print the location of the orange ball if found
    if orange_ball:
        print("Orange ball detected at:", orange_ball)
        if orange_ball in ball_list:
            ball_list.remove(orange_ball)
            ball_list.append(orange_ball)
    else:
        print("No orange ball detected.")

    # Print the list of all detected balls' locations
    print("All detected balls' locations:", ball_list)

    return ball_list

# Usage example:
# frame = cv2.imread('path_to_your_image.jpg')
# mtx, dist = your camera matrix, your distortion coefficients
# ball_locations = detect_balls(frame, mtx, dist)
