import cv2
import numpy as np

def detect_balls(image_path):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the color range for white to light yellow
    lower_color = np.array([0, 0, 200])  # white to light yellow
    upper_color = np.array([40, 100, 255])  # white to light yellow

    # Threshold the HSV image to get only white to light yellow colors
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Bitwise-AND the mask and the original image
    res = cv2.bitwise_and(img, img, mask=mask)

    # Display the image with only white to light yellow colors
    cv2.imshow('white to light yellow colors', res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Call the function with the path to your image
detect_balls('images/image_1.png')