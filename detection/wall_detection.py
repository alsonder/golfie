import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# Load the image
img = cv2.imread('images_w_egg/image_0.png')

# Check if image is loaded
if img is None:
    print("Image not loaded. Check the file path.")
    exit()

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
lines = cv2.HoughLinesP(bw, 1, np.pi/180, threshold=62, minLineLength=0, maxLineGap=5)

# Function to find intersection of two lines
def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denominator == 0:
        return None  # Lines are parallel, no intersection
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
    return (px, py)

# Function to check if a point is within a line segment
def is_within_bounds(pt, line):
    x, y = pt
    x1, y1, x2, y2 = line
    return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

# Visualize lines and intersections
intersections = []
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw lines in green
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            pt = line_intersection(lines[i], lines[j])
            if pt and is_within_bounds(pt, lines[i][0]) and is_within_bounds(pt, lines[j][0]):
                intersections.append(pt)

# Filter intersections based on a histogram threshold
area_size = 10
hist, xedges, yedges = np.histogram2d(
    [pt[0] for pt in intersections],
    [pt[1] for pt in intersections],
    bins=[img.shape[1]//area_size, img.shape[0]//area_size],
    range=[[0, img.shape[1]], [0, img.shape[0]]]
)

filtered_intersections = []
for pt in intersections:
    ix = int((pt[0] - xedges[0]) / area_size)
    iy = int((pt[1] - yedges[0]) / area_size)
    if ix < len(hist) and iy < len(hist[0]) and hist[ix, iy] >= 2:
        filtered_intersections.append(pt)
        cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), -1)  # Draw red circles at intersections

# Apply DBSCAN clustering to intersections
dbscan = DBSCAN(eps=10, min_samples=2)
clusters = dbscan.fit_predict(np.array(filtered_intersections))

# Extract centroids of clusters
clustered_intersections = []
for cluster in np.unique(clusters):
    if cluster != -1:  # Ignore noise points
        points_in_cluster = np.array(filtered_intersections)[clusters == cluster]
        centroid = points_in_cluster.mean(axis=0)
        clustered_intersections.append(centroid)
        print("Cluster centroid (int):", tuple(map(int, centroid)))

# Draw a transparent circle around the last centroid
if clustered_intersections:
    last_point = tuple(map(int, clustered_intersections[-1]))
    overlay = img.copy()
    cv2.circle(overlay, last_point, 37, (255, 0, 255), -1)  # Draw the circle on the overlay
    alpha = 0.5  # Transparency factor
    img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

cv2.imshow('Processed Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
