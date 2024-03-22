import cv2
import numpy as np

# Inspired https://www.geeksforgeeks.org/circle-detection-using-opencv-python/

# Function to detect balls in an image
def detect_balls(frame, mtx, dist):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the color range for white to light yellow
    lower_color = np.array([0, 0, 200])  # white to light yellow
    upper_color = np.array([40, 100, 255])  # white to light yellow

    # Threshold the HSV image to get only white to light yellow colors
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Perform opening to remove small blobs
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out small contours based on area
    min_contour_area = 5  
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

    # Calculate the center of each contour and store in an array
    centers = []  # Initialize centers list
    for cnt in large_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))

        # Draw a small circle at the center
        cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)

    return centers, frame