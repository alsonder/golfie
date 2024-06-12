import cv2

## - imports for main - ##
from find_goal import decide_goal_loc
from find_aruco import detect_aruco
from find_cross import find_and_draw_red_cross
from find_egg import detect_egg
from find_walls import get_line_pixels_and_corners
from fill_cross import fill_polygon
from create_path import nearest_neighbor, a_star_search
from create_grid import gridCreation
from display_grid import visualize_grid

image = cv2.imread('global_values/test_image.png')
#image = cv2.flip(image, 0)
ROW, COL = image.shape[:2]  # image.shape returns (height, width, channels)

# Check if the image has been correctly loaded
if image is not None: 
    print("Image Detection         | Successful")
    print("Width:", ROW, "Height:", COL)
else: print("Failed to load the image")

aruco_location = detect_aruco(image)
find_cross = find_and_draw_red_cross(image)
egg_loc = detect_egg(image)
wall_corner_locations, line_pixels = get_line_pixels_and_corners(image)
goal_location = decide_goal_loc(aruco_location,wall_corner_locations)


ROW, COL = 480,640
grid, weightedGrid = gridCreation(ROW,COL, wall_corner_locations+find_cross+egg_loc)

goal = (round(goal_location[0][1]/2),round(goal_location[0][0]/2))
aruco = (aruco_location[1],aruco_location[0])
path = a_star_search(grid, aruco, (260,320),weightedGrid)
path += a_star_search(grid, path[-1], goal, weightedGrid)


print("\n - - - Some values are - - -")
print("aruco_loc = ", aruco_location)
print("find_cross = ",find_cross[:4]) #4 first values of findcross
print("egg_loc = ", egg_loc[:4]) #4 first values of egg
print("line_pixels = ", line_pixels[:4]) #4 first values of wall
print("goal_loc = ", goal_location)
visualize_grid(grid, aruco, [aruco,goal], path)

# alle blocked cells ligger i "blocked_cells"