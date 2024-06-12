import cv2
import numpy as np
import time

# Global variable to keep track of the last time messages were printed
last_print_time = None

def detect_balls(frame, mtx, dist):
    global last_print_time

    # Current time at the start of the function call
    current_time = time.time()

    # Check if we should print this time
    if last_print_time is None or (current_time - last_print_time) > 2:
        should_print = True
        last_print_time = current_time  # Update the last print time
    else:
        should_print = False

    frame_undistorted = cv2.undistort(frame, mtx, dist)
    hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_gray = clahe.apply(gray)

    gaussian_blur = 1
    blurred_gray = cv2.GaussianBlur(clahe_gray, (gaussian_blur, gaussian_blur), 0)

    param1 = 50  # Gradient value used to handle edge detection in Hough transform
    param2 = 13  # Accumulator threshold for the Hough transform circle detection
    min_radius = 6
    max_radius = 8

    circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, 1, 1,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)
    ball_list = []
    orange_ball_list = []  # List to hold orange balls

    # Expanded HSV range for orange
    lower_orange = np.array([0, 100, 100])  # Extend hue range lower to include more orange shades
    upper_orange = np.array([30, 255, 255])  # Extend hue range higher to include more orange shades

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            ball_hsv_value = hsv[i[1], i[0]]
            if lower_orange[0] <= ball_hsv_value[0] <= upper_orange[0] and \
               lower_orange[1] <= ball_hsv_value[1] <= upper_orange[1] and \
               lower_orange[2] <= ball_hsv_value[2] <= upper_orange[2]:
                orange_ball_list.append([i[0], i[1]])  # Add to orange ball list
                # Print each orange ball's location immediately after detection
                if should_print:
                    print(f"Orange ball: {i[0]},{i[1]}")
            else:
                ball_list.append([i[0], i[1]])  # Add to general ball list

    # Append orange balls at the end of the ball list
    ball_list.extend(orange_ball_list)

    # Conditional printing based on the timer
    if should_print:
        print("No orange ball detected.") if len(orange_ball_list) == 0 else None
        print(f"All detected balls' locations: {ball_list}")
        print(f"Number of balls on the field: {len(ball_list)}")

    return ball_list
