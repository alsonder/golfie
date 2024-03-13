import cv2
import numpy as np

def draw_grid(frame, grid_size):
    # Get the dimensions of the frame
    height, width, _ = frame.shape

    # Draw the horizontal lines
    for i in range(0, height, grid_size):
        cv2.line(frame, (0, i), (width, i), (255, 255, 255), 1)

    # Draw the vertical lines
    for j in range(0, width, grid_size):
        cv2.line(frame, (j, 0), (j, height), (255, 255, 255), 1)

    print(grid_size)
    return frame