import cv2
import numpy as np
from detection.aruco_detection import detect_aruco
from pathfinding.robot_point_calculation import calculate_and_draw_points
from robotposition.robot_control import MotorControl, RobotMovement
import asyncio


import asyncio
import numpy as np
import cv2
from detection.aruco_detection import detect_aruco
from pathfinding.robot_point_calculation import calculate_and_draw_points

async def calibrate_robot_movement_async(stream, mtx, dist, ble_client):
    motor_control = MotorControl(ble_client)  # Assuming this is correctly set up to control your robot
    robot_movement = RobotMovement(motor_control)  # And this manages the movement commands
    robot_movement.start()

    try:
        await ble_client.wait_for_connection()
        TARGET_PROXIMITY_THRESHOLD = 200 # This will be set based on ArUco detection
        ANGLE_TOLERANCE = 0.2  # radians, adjust based on your requirements
        
        projected_point_initialized = False
        projected_point = None
        current_front_point = None
        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_a(60), ble_client.loop)
        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_b(60), ble_client.loop)
        while True:
            frame = stream.get_frame()
            if frame is None:
                print("Failed to get frame")
                continue

            frame_undistorted = cv2.undistort(frame, mtx, dist)
            aruco_corners, aruco_ids, _ = detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)
            
            if aruco_ids is not None and len(aruco_corners) > 0:
                # Process ArUco detection
                frame_undistorted, new_front_point, rear_point = calculate_and_draw_points(frame_undistorted, aruco_corners[0][0])
                
                if not projected_point_initialized:
                    direction_vector = calculate_direction_vector(aruco_corners[0][0])
                    scaling_factor = calculate_scaling_factor(aruco_corners[0][0], marker_real_width_cm=8)
                    projected_point = calculate_projected_point(np.array(new_front_point), direction_vector, distance_cm=50, scaling_factor=scaling_factor)
                    projected_point_initialized = True
                    current_front_point = new_front_point  # Initialize current front point

                cv2.circle(frame_undistorted, projected_point, 5, (0, 0, 255), -1)  # Draw projected point
                
                if projected_point_initialized:
                    # Calculate angle to target
                    angle_to_target = calculate_angle_to_target_radians(current_front_point, rear_point, projected_point)
                    distance_to_target = np.linalg.norm(np.array(current_front_point) - np.array(projected_point))

                    # Turn towards the target if necessary
                if abs(angle_to_target) > ANGLE_TOLERANCE:
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_a(60), ble_client.loop)
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_b(60), ble_client.loop)

                    if angle_to_target > 0:
                        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.turn_right(), ble_client.loop)
                    else:
                        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.turn_left(), ble_client.loop)
                


                else:
                    # If aligned, move forward
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.move_forward(), ble_client.loop)
                print(TARGET_PROXIMITY_THRESHOLD)
                # Additional logic to handle stopping at the target
                if distance_to_target <= TARGET_PROXIMITY_THRESHOLD:
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.stop_movement(), ble_client.loop)
                    print("Target reached.")
                    break
            cv2.imshow('Calibration View', frame_undistorted)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        robot_movement.stop()

    #calibrate_and_detect_balls(stream, mtx, dist)
    #calibrate_robot_movement(stream, mtx, dist, ble_client)


def calibrate_robot_movement(stream, mtx, dist, ble_client):
    asyncio.run(calibrate_robot_movement_async(stream, mtx, dist, ble_client))
def calculate_scaling_factor(corners, marker_real_width_cm):
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    length_vector_mid_pixels = np.linalg.norm(vector_mid)
    scaling_factor = length_vector_mid_pixels / marker_real_width_cm
    return scaling_factor

def calculate_direction_vector(corners):
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    norm_vector_mid = vector_mid / np.linalg.norm(vector_mid)
    return norm_vector_mid

def calculate_projected_point(front_point, direction_vector, distance_cm, scaling_factor):
    front_offset_pixels = distance_cm * scaling_factor
    projected_point = front_point + direction_vector * front_offset_pixels
    return tuple(np.int0(projected_point))

#def navigate_to_ball(stream, mtx, dist, ble_client, target_ball, front_point, rear_point):
    #asyncio.run(navigate_to_ball_async(stream, mtx, dist, ble_client, target_ball, front_point, rear_point))

def calculate_angle_to_target_radians(front_point, rear_point, target_point):
    """
    Calculates the angle in radians between the robot's orientation and the target.
    
    Parameters are the same as before, with the return value now in radians.
    """
    orientation_vector = np.array(front_point) - np.array(rear_point)
    target_vector = np.array(target_point) - np.array(front_point)
    
    unit_orientation_vector = orientation_vector / np.linalg.norm(orientation_vector)
    unit_target_vector = target_vector / np.linalg.norm(target_vector)
    
    dot_product = np.dot(unit_orientation_vector, unit_target_vector)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Use np.clip to handle floating-point errors
    
    # Determine turn direction
    cross_product = np.cross(unit_orientation_vector, unit_target_vector)
    if cross_product > 0:
        return -angle  # Negative for counter-clockwise
    else:
        return angle  # Positive for clockwise

def calculate_scaling_factor(corners, marker_real_width_cm=8):
    """
    Calculate the scaling factor from real-world dimensions (cm) to pixels.

    Parameters:
    - corners: The corners of the detected ArUco marker in the image.
    - marker_real_width_cm: The real-world width of the ArUco marker in centimeters. Defaults to 8 cm.

    Returns:
    - The scaling factor (pixels per centimeter).
    """
    corners = np.squeeze(corners)
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    length_vector_mid_pixels = np.linalg.norm(vector_mid)
    scaling_factor = length_vector_mid_pixels / marker_real_width_cm
    return scaling_factor

