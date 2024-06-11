from wall_detection import*
import cv2
import numpy as np

# Load the image
img = cv2.imread('images_w_egg/image_0.png')

# Check if image is loaded
if img is None:
    print("Image not loaded. Check the file path.")
    exit()


point = clustered_intersections

for pt in clustered_intersections:
    cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 255), -1)  # Display centroids of clusters
    cv2.putText(img, f"({pt[0]}, {pt[1]})", (pt[0] , pt[1] ),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    
def find_middle_point(x1, y1, x2, y2):
    middle_x = (x1 + x2) / 2
    middle_y = (y1 + y2) / 2
    return middle_x, middle_y

points = [(578, 430), (31, 419), (564, 30), (41, 36), (305, 222)]

# Middle point between (41, 36) and (31, 419) large goal
x1, y1 = points[3]
x2, y2 = points[1]
middle_point1 = find_middle_point(x1 + 30, y1, x2 + 30, y2)
print("Large goal:", middle_point1)
cv2.circle(img, (int(middle_point1[0]), int(middle_point1[1])), 5, (255, 0, 0), -1)  # blue circle

# Middle point between (578, 430) and (564, 30) small goal
x1, y1 = points[0]
x2, y2 = points[2]
middle_point2 = find_middle_point(x1 - 30, y1, x2 - 30, y2)
print("Small goal:", middle_point2)
cv2.circle(img, (int(middle_point2[0]), int(middle_point2[1])), 5, (0, 255, 0), -1)  # green circle


cv2.imshow('Clustered Intersections', img)
cv2.waitKey(0)
cv2.destroyAllWindows()