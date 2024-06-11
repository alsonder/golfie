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
    
    corners_array = np.array(aruco_corners)  # This should create a 2D array
    # Make sure that corners_array is not empty and has two dimensions
    if corners_array.ndim != 2 or corners_array.shape[1] != 2:
        raise ValueError("Expected input is a list of tuples with two elements each.")
    centroid_x = np.mean(corners_array[:, 0])  # Mean of all x coordinates
    centroid_y = np.mean(corners_array[:, 1])  # Mean of all y coordinates
    return [(centroid_x, centroid_y)]  # Return a list with a single centroid tuple

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_corners(intersections): #requires touple
    return intersections[:4]

def midpoint(x1, y1, x2, y2):
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def decide_goal_loc(aruco_coor, intersections):
    aruC = aruco_coor# middle of aruco

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
    if (distance(*start_mid_corner2, *intersections[1]) < dist_corner2 and distance(*start_mid_corner2, *intersections[3])!=0):
        dist_corner2 = distance(*start_mid_corner2, *intersections[1])
        mid_corner2 = midpoint(*start_mid_corner2, *intersections[1])
    

    corner1 =  mid_corner1
    corner2 = mid_corner2

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
    
    #print("result1: {}, result2: {}".format(result2, result1))
    print("Goal Detection Successful")

    return [(int(result2[0]), int(result2[1])), (int(result1[0]), int(result1[1]))]
    #example return : [(x1,y1),(x2,y2)]


'''
USE decide_goal_loc(
   aruco_cornervalues as ([x0,y0],[x1,y1],[x2,y2],[x3,y3])
   corners_of_map as ([x0,y0],[x1,y1],[x2,y2],[x3,y3])
)
'''