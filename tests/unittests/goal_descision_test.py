#from ....golfie.pathfinding.AStar.goal_descision import get_aruco_center, distance, get_corners, midpoint, decide_goal_loc
import numpy as np
import math
from PIL import Image, ImageDraw

# what class does
'''
* finds aruco point and 4 corners of the map
* calculates the mid points of the 4 corners to find the exits
* calculates which point is furthest
* returns a touple of endpoint and a point -10x of the endpoint
'''


#impo
def __init__(self):
    self.intersections = []

def set_intersections(self, intersections):
    self.intersections = intersections

def print_intersections(self):
    print("Intersections in Pane:", self.intersections)
#impo

def get_aruco_center(aruco_corners):
    if not aruco_corners:
        return []
    
    # If aruco_corners is a list of tuples like [(x1, y1), (x2, y2), ...]
    corners_array = np.array(aruco_corners)  # This should create a 2D array
    
    # Make sure that corners_array is not empty and has two dimensions
    if corners_array.ndim != 2 or corners_array.shape[1] != 2:
        raise ValueError("Expected input is a list of tuples with two elements each.")
    
    # Calculate the centroid
    centroid_x = np.mean(corners_array[:, 0])  # Mean of all x coordinates
    centroid_y = np.mean(corners_array[:, 1])  # Mean of all y coordinates

    return [(centroid_x, centroid_y)]  # Return a list with a single centroid tuple

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_corners(intersections): #requires touple
    return intersections[:4]

def midpoint(x1, y1, x2, y2):
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def decide_goal_loc(aruco_corners, intersections):
    aruC = midpoint(
        *midpoint(*aruco_corners[0], *aruco_corners[1]),
        *midpoint(*aruco_corners[2], *aruco_corners[3])
    ) # middle of aruco

    #corner1 = midpoint(*intersections[0], *intersections[1])
    #corner2 = midpoint(*intersections[2], *intersections[3])

##
    dist_corner1 = distance(*intersections[0], *intersections[1])
    mid_corner1 = midpoint(*intersections[0], *intersections[1])
    if (distance(*intersections[0], *intersections[2]) < dist_corner1):
        dist_corner1 = distance(*intersections[0], *intersections[2])
        mid_corner1=midpoint(*intersections[0], *intersections[2])
    if (distance(*intersections[0], *intersections[3]) < dist_corner1):
        dist_corner1 = distance(*intersections[0], *intersections[3])
        mid_corner1=midpoint(*intersections[0], *intersections[3])
    
    longcorner1 = distance(*intersections[0], *intersections[1])
    start_mid_corner2 = intersections[1]
    if (distance(*intersections[0], *intersections[2]) > longcorner1):
        longcorner1 = distance(*intersections[0], *intersections[2])
        start_mid_corner2 = intersections[2]
    if (distance(*intersections[0], *intersections[3]) > longcorner1):
        longcorner1 = distance(*intersections[0], *intersections[3])
        start_mid_corner2 = intersections[3]
    
    if (distance(*start_mid_corner2, *intersections[1])!=0):
        dist_corner2 = distance(*start_mid_corner2, *intersections[1])
        mid_corner2 = midpoint(*start_mid_corner2, *intersections[1])
    if (distance(*start_mid_corner2, *intersections[2]) < dist_corner2 and distance(*start_mid_corner2, *intersections[2])!=0):
        dist_corner2 = distance(*start_mid_corner2, *intersections[2])
        mid_corner2 = midpoint(*start_mid_corner2, *intersections[2])
    if (distance(*start_mid_corner2, *intersections[3]) < dist_corner2 and distance(*start_mid_corner2, *intersections[3])!=0):
        dist_corner2 = distance(*start_mid_corner2, *intersections[3])
        mid_corner2 = midpoint(*start_mid_corner2, *intersections[3])
    

    corner1 =  mid_corner1
    corner2 = mid_corner2
    print(corner1)
    print(corner2)
##

    distance1 = distance(aruC[0], aruC[1], *corner1)
    distance2 = distance(aruC[0], aruC[1], *corner2)    

    result1 = corner1 if distance1 > distance2 else corner2
    
    #cal res2 mid mid mid - exit help point finder
    if (result1==corner1): notresult1=corner2
    else: notresult1 = corner1

    result2 = midpoint(*result1, *notresult1)
    result2 = midpoint(*result1, *result2)
    result2 = midpoint(*result1, *result2)
    result2 = midpoint(*result1, *result2)

    return [(result2),(result1)] #example return : [(x1,y1),(x2,y2)]


################### ---- VISUALISATION BELOW ---- ###################


## INPUT VALUES
aruco4 = [(10,20),(10,30),(20,20),(20,30)]
corner4 = [(5,10),(5,50),(110,10),(130,99)]
img_size = (140, 120)

## NCDE

goalloc = decide_goal_loc(aruco4,corner4)
print("goal loc ",goalloc)  # Simplified to just print the result, assuming it's correctly structured

# Create a blank 100x100 white image
image = Image.new("RGB", img_size, "white")
draw = ImageDraw.Draw(image)

# Define the points to draw
points = [midpoint(*midpoint(*aruco4[0], *aruco4[1]), *midpoint(*aruco4[2], *aruco4[3])), # mid of intersec aruco
           midpoint(*corner4[0],*corner4[1]), midpoint(*corner4[2], *corner4[3])] #* unpacks

# Function to draw points
def draw_arucorner(points):
    for x,y in points:
        draw.point((x,y), fill='grey')

def draw_points(points, color='red'):
    radius = 1  
    for x, y in points:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)

def draw_aruco(points, color='blue'):
    radius = 1
    for x, y in points:
        draw.ellipse((x-radius, y-radius, x+radius,y+radius), fill=color)

def draw_corner_points(points, color='grey'):
    radius = 1  
    for x, y in points:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
        
def draw_goal(points, color='green'):
    radius = 2  
    for x, y in points:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)

# Define special coordinates that need to be colored differently

draw_arucorner(aruco4)
draw_points(points, 'red')
draw_aruco(points[:1])
draw_corner_points(corner4)
draw_goal(goalloc)

# Save the image
image.save('points_visualization.png')

# Show the image
image.show()