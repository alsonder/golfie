import cv2
import numpy as np
from find_goal import decide_goal_loc
from find_aruco import detect_aruco
from find_cross import find_and_draw_red_cross
from find_egg import detect_egg
from find_walls import get_line_pixels_from_image


""" This module is simply supposed to gather all the information from the rest of the globel values
acting as a single accesspoint. It doesn't take any input, but will give you a list containing all  """


def generate_blocked_cells():
    
    image_path = "global_values/test_image.png" #Needs to be changed for real use
    frame = cv2.imread(image_path)
    blocked_cells_list = []

    blocked_cells_list.append(get_line_pixels_from_image(frame))
    blocked_cells_list.append(find_and_draw_red_cross(frame))
    blocked_cells_list.append(detect_egg(frame))

    return blocked_cells_list

print(generate_blocked_cells())