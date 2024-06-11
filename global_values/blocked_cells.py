import cv2
import numpy as np


""" This module is simply supposed to gather all the information from the rest of the globel values
acting as a single accesspoint. It doesn't take any input, but will give you a list containing all  """


def generate_blocked_cells():
    
    image_path = "global_values/test_image.png" #Needs to be changed for real use
    frame = cv2.imread(image_path)
    blocked_cells_list = []

    #Call all the functions and add to the list
    

