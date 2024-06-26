import numpy as np
import cv2

"""This function will get the corners of the aruco markers, and use it to calculate the front
    mid point on the robot, this will be the point where all the calculations in regards to 
    navigating the robot, the front point is physically located 1.5 cm ahead of the marker
    and this functions purpose is to determine the orientation of the robot, as well as
    the front point, now the front point is calculated by using the fact that we know
    the aruco marker is 8x8 cm, and we want to project the front 1.5 cm ahead. it then 
    calculates the scaling factor, how long is 8 cm in pixels, it then multiplies the 
    scaling factor with the front point coordinates, and draws both this point as well
    as the rear mid point of the robot"""

def calculate_and_draw_points(frame, corners):
    # Assure corners shape is (4, 2)
    corners = np.squeeze(corners)

    # Define vectors based on corners
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]

    # Mid rear point
    mid_rear = (corners[3] + corners[2]) / 2

    # Calculate the middle vector as the midpoint between the left and right vectors
    vector_mid = (vector_left + vector_right) / 2
    mid_front = (corners[0] + corners[1]) / 2

    # Calculate the scaling factor based on the length of the middle vector
    # This assumes the marker is 8cm in width, change if marker size is changed
    marker_real_width_cm = 8
    length_vector_mid_pixels = np.linalg.norm(vector_mid)
    scaling_factor = length_vector_mid_pixels / marker_real_width_cm

    # Calculate the front point offset by 1.5cm from the midpoint front in the direction of vector_mid
    front_offset_cm = 1.5
    front_offset_pixels = front_offset_cm * scaling_factor

    # The front point is calculated by adding the normalized mid vector scaled by the front_offset_pixels to the mid front point
    norm_vector_mid = vector_mid / np.linalg.norm(vector_mid)
    front_point = mid_front + norm_vector_mid * front_offset_pixels

    # Convert points to integer for drawing
    mid_rear_int = tuple(np.int0(mid_rear))
    front_point_int = tuple(np.int0(front_point))

    # Draw the mid rear point in green and the front point in blue for visualization
    cv2.circle(frame, mid_rear_int, 5, (0, 255, 0), -1)  # Green for mid rear point
    cv2.circle(frame, front_point_int, 5, (255, 0, 0), -1)  # Blue for front point

    return frame, front_point_int, mid_rear_int
