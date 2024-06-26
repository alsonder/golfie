import cv2
import numpy as np

def detect_balls(frame, mtx, dist):
    # Undistort the frame using the given camera matrix and distortion coefficients
    frame_undistorted = cv2.undistort(frame, mtx, dist)
    hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_gray = clahe.apply(gray)

    # Gaussian Blur to reduce noise
    gaussian_blur = 1
    blurred_gray = cv2.GaussianBlur(clahe_gray, (gaussian_blur * 2 + 1, gaussian_blur * 2 + 1), 0)

    # Hough Circles detection parameters
    param1 = 14  # Gradient value used to handle edge detection in Hough transform
    param2 = 14  # Accumulator threshold for the Hough transform circle detection
    min_radius = 7
    max_radius = 9
    # param1 = 50  # Gradient value used to handle edge detection in Hough transform
    # param2 = 13  # Accumulator threshold for the Hough transform circle detection
    # min_radius = 6
    # max_radius = 7

    # Detect circles
    circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, 1, 15,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)

    ball_list = []
    orange_ball_list = []  # List to hold orange balls

    # HSV range for orange
    lower_orange = np.array([0, 100, 100])  # Extend hue range lower to include more orange shades
    upper_orange = np.array([30, 255, 255])  # Extend hue range higher to include more orange shades

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            ball_hsv_value = hsv[i[1], i[0]]
            if lower_orange[0] <= ball_hsv_value[0] <= upper_orange[0] and \
               lower_orange[1] <= ball_hsv_value[1] <= upper_orange[1] and \
               lower_orange[2] <= ball_hsv_value[2] <= upper_orange[2]:
                orange_ball_list.append((i[0], i[1]))  # Add to orange ball list as tuple
            else:
                ball_list.append((i[0], i[1]))  # Add to general ball list as tuple

    # Append orange balls at the end of the ball list
    ball_list.extend(orange_ball_list)

    #print("Detected balls:", ball_list)
    return ball_list


# Usage in your existing code, e.g., within a loop where you capture frames from the camera:
# cap = cv2.VideoCapture(0)
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
#     ball_locations = detect_balls(frame, mtx, dist)
#     for x, y in ball_locations:
#         cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)  # Draw circles on detected balls
#     cv2.imshow("Detected Balls", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()

