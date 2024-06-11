import cv2

## imports for main
from find_goal import decide_goal_loc
from find_aruco import detect_aruco
## imports for main

image = cv2.imread('global_values/test_image.png')

# Check if the image has been correctly loaded
if image is not None: print("Image Detection Successful")
else: print("Failed to load the image")

# Temporary Values
filler_aruco = ([15,15]) # aruco
filler_wall_corners = ([10,10],[15,300],[400,290],[404,308]) # wall corners

aruco_location = detect_aruco(image)
goal_location = decide_goal_loc(filler_aruco,filler_wall_corners)



print(" - - - Some values are - - -")
print("aruco_loc = ", aruco_location)
print("goal_loc = ", goal_location)