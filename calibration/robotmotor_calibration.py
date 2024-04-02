import cv2
import numpy as np
from detection.aruco_detection import detect_aruco
from pathfinding.robot_point_calculation import calculate_and_draw_points
from robotposition.robot_control import MotorControl

"""The point of this part is to have a way to calibrate the pwm signal sent to the motors, if this is not done, various inacurracies will be present
The total voltag sent to the motors are a function of the pwm signal sent, and this depends on how much power the batteries have. THis is a safeguard 
to achieve less adjustments on competition day, time spent on adjusting to the line its sopposed to go, can be avoided with this"""

def calibrate_robot_movement(stream, mtx, dist, ble_client):
    motor_control = MotorControl(ble_client)  # BLE motor control setup, MotorControl object instance

    while True:
        frame = stream.get_frame()
        if frame is None:
            print("Failed to get frame")
            continue  # Skip rest of the loop if no frame
        
        frame_undistorted = cv2.undistort(frame, mtx, dist)
        front_point= None
        # Detect ArUco and calculate the front point for the first detected marker
        aruco_corners, aruco_ids, _ = detect_aruco(stream, mtx, dist, markerLength=0.08)
        if aruco_ids is not None and len(aruco_corners) > 0:  # Check if at least one marker is detected
            # Ensure aruco_corners[0] is properly structured for calculate_and_draw_points
            frame_undistorted, front_point = calculate_and_draw_points(frame_undistorted, aruco_corners[0][0])

            # Ensure we're passing the correct structure to calculate_direction_vector
            direction_vector = calculate_direction_vector(aruco_corners[0][0])
            scaling_factor = calculate_scaling_factor(aruco_corners[0][0], 8)  # Assuming markerLength=0.08 m ArUco marker
            projected_point = calculate_projected_point(np.array(front_point), direction_vector, 100, scaling_factor)
            setFront_point = front_point
            
            # Draw the projected point and a new line from the front point to the projected point
            cv2.circle(frame_undistorted, projected_point, 5, (0, 255, 0), -1)
            cv2.line(frame_undistorted, setFront_point, projected_point, (255, 0, 0), 2) 
        else:
            print("No ArUco markers detected, unable to calibrate robot movement.")

        cv2.imshow('Calibration', frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# For calibration we calculate the pixel scaling ratio constant, to multiply with the measurement in cm
# unit is now in pixels
def calculate_scaling_factor(corners, marker_real_width_cm):
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    length_vector_mid_pixels = np.linalg.norm(vector_mid)
    scaling_factor = length_vector_mid_pixels / marker_real_width_cm
    return scaling_factor


# Calculate the direction vector of the middle vector to get orientation
def calculate_direction_vector(corners):
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    norm_vector_mid = vector_mid / np.linalg.norm(vector_mid)
    return norm_vector_mid

# Projected point 100 cm in a straight line from the direction vector of the aruco marker
def calculate_projected_point(front_point, direction_vector, distance_cm, scaling_factor):
    front_offset_pixels = distance_cm * scaling_factor
    projected_point = front_point + direction_vector * front_offset_pixels
    return tuple(np.int0(projected_point))
