import cv2
import numpy as np
from detection.aruco_detection import detect_aruco
from pathfinding.robot_point_calculation import calculate_and_draw_points
from robotposition.robot_control import MotorControl, RobotMovement
import asyncio


async def calibrate_robot_movement_async(stream, mtx, dist, ble_client):
    motor_control = MotorControl(ble_client)
    robot_movement = RobotMovement(motor_control)
    robot_movement.start()

    try:
        projected_point_initialized = False
        current_front_point = None

        while True:
            frame = stream.get_frame()
            if frame is None:
                print("Failed to get frame")
                continue

            frame_undistorted = cv2.undistort(frame, mtx, dist)
            aruco_corners, aruco_ids, _ = detect_aruco(stream, mtx, dist, markerLength=0.08)

            if aruco_ids is not None and len(aruco_corners) > 0:
                # Process each detected ArUco marker
                for corner_group in aruco_corners:
                    # Get the frame with points drawn and the current front point
                    frame_with_points, current_front_point = calculate_and_draw_points(frame_undistorted, corner_group[0])
                    if not projected_point_initialized:
                        direction_vector = calculate_direction_vector(corner_group[0])
                        scaling_factor = calculate_scaling_factor(corner_group[0], 8)  # Assuming marker width of 8cm
                        projected_point = calculate_projected_point(np.array(current_front_point), direction_vector, 50, scaling_factor)  
                        projected_point_initialized = True

                        # Start moving towards the projected point
                        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.move_forward(), ble_client.loop)

                    # Check if the current front point is approximately at the projected point
                    if projected_point_initialized and current_front_point and abs(current_front_point[0] - projected_point[0]) <= 10:  # Threshold of 10 pixels
                        asyncio.run_coroutine_threadsafe(robot_movement.motor_control.stop_movement(), ble_client.loop)
                        print("Robot has reached the projected point.")
                        break

            cv2.imshow('Calibration', frame_with_points)  # Use the frame with points drawn
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        robot_movement.stop()

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
