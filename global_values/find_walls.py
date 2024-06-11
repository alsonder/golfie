import cv2
import numpy as np

def get_line_pixels_from_image(frame):
    # Load the image
    img = cv2.imread(frame)

    # Check if the image is loaded
    if img is None:
        print("Image not loaded. Check the file path.")
        return []

    # Define the color range for red
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([50, 50, 255])

    # Create a mask for red pixels
    mask = cv2.inRange(img, lower_red, upper_red)

    # Filter the image using the mask
    red_img = cv2.bitwise_and(img, img, mask=mask)

    # Convert to grayscale
    gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)

    # Convert grayscale image to black and white
    _, bw = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    # Define a kernel for the morphological operations
    kernel = np.ones((5,5), np.uint8)

    # Perform dilation, closing, and erosion
    bw = cv2.dilate(bw, kernel, iterations=1)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
    bw = cv2.erode(bw, kernel, iterations=1)
    bw = cv2.dilate(bw, kernel, iterations=1)

    # Detect lines using HoughLinesP
    lines = cv2.HoughLinesP(bw, 1, np.pi/180, threshold=62, minLineLength=10, maxLineGap=20)

    # Function to get points from a line using Bresenham's algorithm
    def get_line_pixels(x1, y1, x2, y2):
        line_pixels = []
        dx = abs(x2 - x1)
        dy = -abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx + dy
        while True:
            line_pixels.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x1 += sx
            if e2 <= dx:
                err += dx
                y1 += sy
        return line_pixels

    # Collect all pixels in all detected lines
    all_line_pixels = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            pixels = get_line_pixels(x1, y1, x2, y2)
            all_line_pixels.extend(pixels)

    return all_line_pixels

# Example usage
image_path = 'global_values/test_image.png'
line_pixels = get_line_pixels_from_image(image_path)
print(line_pixels)
