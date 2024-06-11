import cv2
from find_goal import decide_goal_loc
image = cv2.imread('global_values/test_image.png')

# Check if the image has been correctly loaded
if image is not None: print("I see the image")
else: print("Failed to load the image")

# Temporary Values
filler_aruco = ([15,15])
filler_wall_corners = ([10,10],[15,300],[400,290],[404,308])

decide_goal_loc(filler_aruco,filler_wall_corners)
