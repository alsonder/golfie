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
                #print("Intersection detected at:", pt)  # Debug print

# Define the size of the area for the histogram
area_size = 10

# Create a 2D histogram of the intersection points
hist, xedges, yedges = np.histogram2d(
    [pt[0] for pt in intersections],
    [pt[1] for pt in intersections],
    bins=[img.shape[1]//area_size, img.shape[0]//area_size],
    range=[[0, img.shape[1]], [0, img.shape[0]]]
)

#print("Histogram of intersections:", hist)  # Debug print

# Filter out the points that have less than x intersections in the same area
x_threshold = 2  # Change this to the desired number of intersections
filtered_intersections = []
for pt in intersections:
    ix = int((pt[0] - xedges[0]) / area_size)
    iy = int((pt[1] - yedges[0]) / area_size)
    if ix < len(hist) and iy < len(hist[0]) and hist[ix, iy] >= x_threshold:
        filtered_intersections.append(pt)
        #print("Filtered intersection:", pt)  # Debug print

# Visualize the filtered intersections
for pt in filtered_intersections:
    cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), -1)  # Draw intersections

# Convert list of intersections to a numpy array for DBSCAN
intersection_points = np.array(intersections)

# Apply DBSCAN clustering
dbscan = DBSCAN(eps=10, min_samples=2)  # eps is the maximum distance between two samples for them to be considered as in the same neighborhood
clusters = dbscan.fit_predict(intersection_points)

# Extract the centroids of clusters
clustered_intersections = []
for cluster in np.unique(clusters):
    if cluster != -1:  # Ignore noise points
        points_in_cluster = intersection_points[clusters == cluster]
        centroid = points_in_cluster.mean(axis=0)
        centroid_int = (int(centroid[0]), int(centroid[1]))  # Convert coordinates to integer
        clustered_intersections.append(centroid_int)
        # Print the coordinates of the centroid as integers
        #print("Cluster centroid (int):", centroid_int)
    print("CC", clustered_intersections[:4])

#addit impo
    def __init__(self):
        self.clustered_intersections = []

    def process_intersections(self):
        self.clustered_intersections = clustered_intersections[:4]
#addit impo

# Display the results
for pt in clustered_intersections:
    cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 255), -1)  # Display centroids of clusters

cv2.imshow('Clustered Intersections', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
