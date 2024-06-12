import asyncio
import numpy as np
import cv2
from robotposition.robot_control import MotorControl, RobotMovement
from detection.aruco_detection import detect_aruco
from pathfinding.robot_point_calculation import calculate_and_draw_points
from detection.balls_confirmation import BallConfirmation

async def navigate_to_ball_async(stream, mtx, dist, ble_client, target_ball, front_point, rear_point):
    motor_control = MotorControl(ble_client)
    robot_movement = RobotMovement(motor_control)
    robot_movement.start()
    ANGLE_TOLERANCE = 0.2 # angle tolerance 11.5 degrees, adjust if eternal adjust loop
    try:
        await ble_client.wait_for_connection()
        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.suction_on(), ble_client.loop)

        # Angle in radians
        angle_to_target = calculate_angle_to_target_radians(front_point, rear_point, target_ball)
        distance_to_target = np.linalg.norm(np.array(front_point) - np.array(target_ball))
        frame = stream.get_frame()
        frame_undistorted = cv2.undistort(frame, mtx, dist)
        aruco_corners, aruco_ids, _ = detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)
        if aruco_ids is not None and len(aruco_corners) > 0:
            TARGET_PROXIMITY_THRESHOLD = 2 * calculate_scaling_factor(aruco_corners, marker_real_width_cm=8)  # 2 cm from the ball
        else:
            print("No ArUco markers detected. Cannot calculate TARGET_PROXIMITY_THRESHOLD.")
            return 
        while distance_to_target > TARGET_PROXIMITY_THRESHOLD:  # Assuming some proximity threshold
            # Turn towards the target if necessary
            if abs(angle_to_target) > ANGLE_TOLERANCE:  # Assuming some tolerance for the angle
                asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_a(60), ble_client.loop)
                asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_b(60), ble_client.loop)

                if angle_to_target > 0:
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.turn_right(), ble_client.loop)

                else:
                    asyncio.run_coroutine_threadsafe(robot_movement.motor_control.turn_left(), ble_client.loop)

                asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_a(110), ble_client.loop)
                asyncio.run_coroutine_threadsafe(robot_movement.motor_control.set_pwm_motor_b(110), ble_client.loop)

            # Move forward
            asyncio.run_coroutine_threadsafe(robot_movement.motor_control.move_forward(), ble_client.loop)

            # Update the position and angle based on the latest frame
            frame = stream.get_frame()
            if frame is None:
                print("Failed to get frame")
                continue

            frame_undistorted = cv2.undistort(frame, mtx, dist)
            aruco_corners, aruco_ids, _ = detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)
            TARGET_PROXIMITY_THRESHOLD = 2*calculate_scaling_factor(aruco_corners, marker_real_width_cm=8) #2 cm from th eball
            if aruco_ids is not None and len(aruco_corners) > 0:
                for corner_group in aruco_corners:
                    frame_undistorted, current_front_point, current_rear_point = calculate_and_draw_points(frame_undistorted, corner_group[0])
                    angle_to_target = calculate_angle_to_target_radians(current_front_point, current_rear_point, target_ball)
                    distance_to_target = np.linalg.norm(np.array(current_front_point) - np.array(target_ball))

            # Optionally, slow down as the robot approaches the target by adjusting PWM values
            # if distance_to_target < SLOW_DOWN_THRESHOLD:
            #     await motor_control.adjust_speed_for_approach()

            # Break out of the loop once the robot is within the TARGET_PROXIMITY_THRESHOLD
            if distance_to_target <= TARGET_PROXIMITY_THRESHOLD:
                asyncio.run_coroutine_threadsafe(robot_movement.motor_control.stop_movement(), ble_client.loop)

                print("Target reached.")
                break

    finally:
        robot_movement.stop()
        
def navigate_to_ball(stream, mtx, dist, ble_client, target_ball, front_point, rear_point):
    asyncio.run(navigate_to_ball_async(stream, mtx, dist, ble_client, target_ball, front_point, rear_point))

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


