import asyncio
import numpy as np
import cv2
import time
from robotposition.robot_control import MotorControl, RobotMovement
from detection.aruco_detection import detect_aruco
from detection.ball_detection import detect_balls
from pathfinding.robot_point_calculation import calculate_and_draw_points
from detection.balls_confirmation import BallConfirmation
from livestream import livestream
from pathfinding.shortest_path import find_closest_ball

def navigate_to_ball(stream, mtx, dist, ble_client, closest_ball, front_point, rear_point, done_callback):
    print("Entered navigate_to_ball function.")

    motor_control = MotorControl(ble_client)
    robot_movement = RobotMovement(motor_control)  # Movement commands
    robot_movement.start()
    ANGLE_THRESHOLD = 0.2
    MOVEMENT_DELAY = 0.5  # Delay in seconds to throttle command sending

    try:
        print("Sending initial commands.")
        asyncio.run_coroutine_threadsafe(motor_control.suction_on(), ble_client.loop).result()
        asyncio.run_coroutine_threadsafe(motor_control.set_pwm_motor_a(60), ble_client.loop).result()
        asyncio.run_coroutine_threadsafe(motor_control.set_pwm_motor_b(60), ble_client.loop).result()

        last_command_time = time.time()
        last_command = None

        while True:
            print("Navigating loop started.")
            try:
                frame = stream.get_frame()
                if frame is None:
                    print("Failed to get frame")
                    break
            except Exception as e:
                print(f"Error getting frame: {e}")
                break

            frame_undistorted = cv2.undistort(frame, mtx, dist)
            print("Frame undistorted.")

            aruco_corners, aruco_ids, _ = detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)
            if aruco_ids is not None and len(aruco_corners) > 0:
                for corner_group in aruco_corners:
                    if corner_group is not None and len(corner_group) > 0 and len(corner_group[0]) >= 4:
                        frame_undistorted, front_point, rear_point = calculate_and_draw_points(frame_undistorted, corner_group[0])
            _, current_front_point, _ = calculate_and_draw_points(frame_undistorted, aruco_corners[0])
            scaling_factor = calculate_scaling_factor(aruco_corners[0])
            print(f"ArUco detection done. front_point: {front_point}, rear_point: {rear_point}")

            direction_to_ball = np.array(closest_ball) - np.array(current_front_point)
            direction_vector = direction_to_ball / np.linalg.norm(direction_to_ball)
            if current_front_point is not None and rear_point is not None and closest_ball is not None:
                angle_to_target = calculate_angle_to_target_radians(current_front_point, rear_point, closest_ball)
                print(f"Angle to target calculated: {angle_to_target}")

            corners_array = aruco_corners[0] if aruco_corners and len(aruco_corners) > 0 else None

            if corners_array is not None and len(corners_array) > 0 and all(len(corner) == 4 for corner in corners_array):
                first_marker_corners = corners_array[0]
                stopping_zone = calculate_stopping_zone(first_marker_corners, scaling_factor, front_offset_cm=4, rear_offset_cm=9.5)
                print(f"Stopping zone calculated: {stopping_zone}")

            current_time = time.time()
            if abs(angle_to_target) > ANGLE_THRESHOLD:
                if angle_to_target > 0 and last_command != "left" and (current_time - last_command_time) > MOVEMENT_DELAY:
                    print(f"Turning left: angle_to_target={angle_to_target}, last_command_time={last_command_time}, current_time={current_time}")
                    asyncio.run_coroutine_threadsafe(motor_control.turn_left(), ble_client.loop).result()
                    last_command = "left"
                    last_command_time = current_time
                elif angle_to_target <= 0 and last_command != "right" and (current_time - last_command_time) > MOVEMENT_DELAY:
                    print(f"Turning right: angle_to_target={angle_to_target}, last_command_time={last_command_time}, current_time={current_time}")
                    asyncio.run_coroutine_threadsafe(motor_control.turn_right(), ble_client.loop).result()
                    last_command = "right"
                    last_command_time = current_time
            else:
                if last_command != "forward" and (current_time - last_command_time) > MOVEMENT_DELAY:
                    print(f"Moving forward: angle_to_target={angle_to_target}, last_command_time={last_command_time}, current_time={current_time}")
                    asyncio.run_coroutine_threadsafe(motor_control.move_forward(), ble_client.loop).result()
                    last_command = "forward"
                    last_command_time = current_time

            ball_in_stopping_zone = in_stopping_zone(closest_ball, stopping_zone)
            if ball_in_stopping_zone:
                print(f"Ball in stopping zone, stopping movement: closest_ball={closest_ball}, stopping_zone={stopping_zone}")
                asyncio.run_coroutine_threadsafe(motor_control.stop_movement(), ble_client.loop).result()
                asyncio.run_coroutine_threadsafe(motor_control.suction_on(), ble_client.loop).result()
                time.sleep(2)
                break  # Break out of the while loop once the ball is picked up

            cv2.imshow('Navigation View', frame_undistorted)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(MOVEMENT_DELAY)

    except Exception as e:
        print(f"An error occurred during navigation: {e}")
    finally:
        print("Exiting navigate_to_ball function.")
        if done_callback:
            done_callback()

def in_stopping_zone(ball_position, stopping_zone):
    front_left = np.array(stopping_zone['front_left'])
    front_right = np.array(stopping_zone['front_right'])
    rear_left = np.array(stopping_zone['rear_left'])
    rear_right = np.array(stopping_zone['rear_right'])

    ball_pos = np.array(ball_position)

    is_right_of_left_boundary = np.cross(rear_left - front_left, ball_pos - front_left) <= 0
    is_left_of_right_boundary = np.cross(rear_right - front_right, ball_pos - front_right) >= 0
    is_below_top_boundary = np.cross(front_right - front_left, ball_pos - front_left) <= 0
    is_above_bottom_boundary = np.cross(rear_right - rear_left, ball_pos - rear_left) >= 0

    return is_right_of_left_boundary and is_left_of_right_boundary and is_below_top_boundary and is_above_bottom_boundary

def calculate_angle_to_target_radians(front_point, rear_point, target_point):
    orientation_vector = np.array(front_point) - np.array(rear_point)
    target_vector = np.array(target_point) - np.array(front_point)
    
    unit_orientation_vector = orientation_vector / np.linalg.norm(orientation_vector)
    unit_target_vector = target_vector / np.linalg.norm(target_vector)
    
    dot_product = np.dot(unit_orientation_vector, unit_target_vector)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
    
    cross_product = np.cross(unit_orientation_vector, unit_target_vector)
    if cross_product > 0:
        return -angle
    else:
        return angle

def draw_stopping_zone(frame, stopping_zone):
    cv2.line(frame, stopping_zone['front_left'], stopping_zone['front_right'], (0, 255, 0), 2)
    cv2.line(frame, stopping_zone['front_right'], stopping_zone['rear_right'], (0, 255, 0), 2)
    cv2.line(frame, stopping_zone['rear_right'], stopping_zone['rear_left'], (0, 255, 0), 2)
    cv2.line(frame, stopping_zone['rear_left'], stopping_zone['front_left'], (0, 255, 0), 2)
    return frame

def calculate_scaling_factor(corners, marker_real_width_cm=8):
    corners = np.squeeze(corners)
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    length_vector_mid_pixels = np.linalg.norm(vector_mid)
    scaling_factor = length_vector_mid_pixels / marker_real_width_cm
    return scaling_factor

def calculate_projected_point(front_point, direction_vector, distance_cm, scaling_factor):
    front_offset_pixels = distance_cm * scaling_factor
    projected_point = front_point + direction_vector * front_offset_pixels
    return tuple(np.int0(projected_point))

def calculate_direction_vector(corners):
    vector_left = corners[0] - corners[3]
    vector_right = corners[1] - corners[2]
    vector_mid = (vector_left + vector_right) / 2
    norm_vector_mid = vector_mid / np.linalg.norm(vector_mid)
    return norm_vector_mid

def calculate_stopping_zone(corners, scaling_factor, front_offset_cm=4, rear_offset_cm=9.5):
    front_offset_pixels = front_offset_cm * scaling_factor
    rear_offset_pixels = rear_offset_cm * scaling_factor

    direction_vector = (np.array(corners[0]) + np.array(corners[1])) / 2 - (np.array(corners[3]) + np.array(corners[2])) / 2
    direction_vector /= np.linalg.norm(direction_vector)
    
    side_vector = np.array([direction_vector[1], -direction_vector[0]])

    front_left = np.array(corners[0]) + direction_vector * front_offset_pixels
    front_right = np.array(corners[1]) + direction_vector * front_offset_pixels
    rear_left = np.array(corners[3]) + direction_vector * rear_offset_pixels
    rear_right = np.array(corners[2]) + direction_vector * rear_offset_pixels

    stopping_zone = {
        'front_left': tuple(np.int0(front_left)),
        'front_right': tuple(np.int0(front_right)),
        'rear_left': tuple(np.int0(rear_left)),
        'rear_right': tuple(np.int0(rear_right))
    }

    return stopping_zone


async def simple_navigate_to_ball(ble_client, closest_ball, front_point, rear_point, startup, finish):
    ANGLE_THRESHOLD = 0.25
    MOVEMENT_DELAY = 0.075  # Delay in seconds to throttle command sending
    RUN_DURATION = .15  # Run for 1 second
    POSITION_THRESHOLD = 15  # Distance in pixels to consider the ball as reached

    motor_control = MotorControl(ble_client)
    last_command_time = time.time()
    start_time = time.time()
    last_command = None
    
    closest_ball = (closest_ball[1], closest_ball[0])
    front_point = (front_point[1], front_point[0])
    rear_point = (rear_point[1], rear_point[0])
    
    print(f"from: {front_point} to  {closest_ball}")
    
    if startup:
        print("Sending initial commands.")
        asyncio.run_coroutine_threadsafe(motor_control.blow_on(), ble_client.loop).result()
        asyncio.run_coroutine_threadsafe(motor_control.set_pwm_motor_a(90), ble_client.loop).result()
        asyncio.run_coroutine_threadsafe(motor_control.set_pwm_motor_b(90), ble_client.loop).result()

    def is_close_to_target(current, target, threshold):
        return np.linalg.norm(np.array(current) - np.array(target)) < threshold

    while time.time() - start_time < RUN_DURATION:
        angle_to_target = calculate_angle_to_target_radians(front_point, rear_point, closest_ball)
        current_time = time.time()

        if abs(angle_to_target) > ANGLE_THRESHOLD:
            if angle_to_target > 0 and last_command != "right" and (current_time - last_command_time) > MOVEMENT_DELAY:
                asyncio.run_coroutine_threadsafe(motor_control.turn_right(), ble_client.loop).result()
                last_command = "right"
                last_command_time = current_time
            elif angle_to_target < 0 and last_command != "left" and (current_time - last_command_time) > MOVEMENT_DELAY:
                asyncio.run_coroutine_threadsafe(motor_control.turn_left(), ble_client.loop).result()
                last_command = "left"
                last_command_time = current_time
        else:
            if last_command != "forward" and (current_time - last_command_time) > MOVEMENT_DELAY:
                asyncio.run_coroutine_threadsafe(motor_control.move_forward(), ble_client.loop).result()
                last_command = "forward"
                last_command_time = current_time

        if is_close_to_target(front_point, closest_ball, POSITION_THRESHOLD):
            asyncio.run_coroutine_threadsafe(motor_control.stop_movement(), ble_client.loop).result()
            if(finish):
                asyncio.run_coroutine_threadsafe(motor_control.suction_on(), ble_client.loop).result()
            else:
                asyncio.run_coroutine_threadsafe(motor_control.blow_on(), ble_client.loop).result()

            
            asyncio.sleep(5)
            return True, closest_ball  # Return True if the ball is reached

        asyncio.sleep(MOVEMENT_DELAY)

    return False, closest_ball  # Return False if the robot did not reach the ball in the given time