import cv2
import numpy as np

def find_and_draw_red_cross(image):
    # Step 1: Read the image
    if image is None:
        raise ValueError("Image not found or unable to open")

    # Step 2: Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Step 3: Define the range for the red color in HSV
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Step 4: Create a mask for the red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Step 5: Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Step 6: Filter for cross shape
    cross_contour = None
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the contour has 12 points (indicative of a cross with rectangular segments)
        if len(approx) == 12:
            cross_contour = approx
            break

    if cross_contour is None:
        raise ValueError("Cross not found in the image")

    # Step 7: Draw the contour on the image
    cv2.drawContours(image, [cross_contour], -1, (0, 255, 0), 2)

    # Step 8: Show the image
    cv2.imshow('Image with Cross Outline', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Extract and return coordinates
    coordinates = [(point[0][0], point[0][1]) for point in cross_contour]
    return coordinates

# Example usage
image_path = "global_values/test_image.png"
image = cv2.imread(image_path)
cross_coordinates = find_and_draw_red_cross(image)
print("Cross coordinates:", cross_coordinates)