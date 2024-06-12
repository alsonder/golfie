import cv2
## - imports for main - ##
from find_goal import decide_goal_loc
from find_aruco import detect_aruco
from find_cross import find_and_draw_red_cross
from find_egg import detect_egg
from find_walls import get_line_pixels_and_corners
## - imports for main - ##
def get_map_values(image):
    # Check if the image has been correctly loaded
    if image is not None: print("Image Detection       | Successful")
    else: print("Failed to load the image")

    # Temporary Values
    filler_wall_corners = ([10,10],[15,300],[400,290],[404,308]) # wall corners

    aruco_location = detect_aruco(image)
    find_cross = find_and_draw_red_cross(image)
    egg_loc = detect_egg(image)
    line_pixels = get_line_pixels_and_corners(image)
    goal_location = decide_goal_loc(aruco_location,filler_wall_corners)


    print("\n - - - Some values are - - -")
    print("aruco_loc = ", aruco_location)
    print("find_cross = ",find_cross[:4]) #4 first values of findcross
    print("egg_loc = ", egg_loc[:4]) #4 first values of egg
    print("line_pixels = ", line_pixels[:4]) #4 first values of wall
    print("goal_loc = ", goal_location)
    print(" - - - - - - - - - - - - - -\n")

    return aruco_location, find_cross, egg_loc, line_pixels, goal_location