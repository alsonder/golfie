import cv2
import numpy as np
from sklearn.cluster import DBSCAN

def get_line_pixels_and_corners(frame):
    # Load the image
    img = frame

    # Check if the image is loaded
    if img is None:
        print("Image not loaded. Check the frame.")
        return [], []

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

    # Find intersections
    intersections = []
    if lines is not None:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                pt = line_intersection(lines[i], lines[j])
                if pt and is_within_bounds(pt, lines[i][0]) and is_within_bounds(pt, lines[j][0]):
                    intersections.append(pt)

    # Apply DBSCAN clustering to intersections
    clustered_intersections = []
    if intersections:
        dbscan = DBSCAN(eps=10, min_samples=2)
        clusters = dbscan.fit_predict(np.array(intersections))
        for cluster in np.unique(clusters):
            if cluster != -1:  # Ignore noise points
                points_in_cluster = np.array(intersections)[clusters == cluster]
                centroid = points_in_cluster.mean(axis=0)
                clustered_intersections.append(centroid)

    print("Wall Detection and Corner Identification Successful")
    return all_line_pixels, clustered_intersections

def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denominator == 0:
        return None  # Lines are parallel, no intersection
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
    return (px, py)

def is_within_bounds(pt, line):
    x, y = pt
    x1, y1, x2, y2 = line
    return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

# Example usage
image = cv2.imread('global_values/test_image.png')
line_pixels, corners = get_line_pixels_and_corners(image)
print("Line Pixels:", line_pixels)
print("Corners:", corners)
