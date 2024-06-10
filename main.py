import cv2
import time
from livestream import livestream
from calibration import camera_calibration
from detection import aruco_detection
from livedata.livedata import LiveData
from detection.ball_detection import detect_balls
from detection.balls_confirmation import BallConfirmation
from pathfinding.robot_point_calculation import calculate_and_draw_points
from pathfinding.shortest_path import find_closest_ball
from ball_image_calibration import calibrate_and_detect_balls
from connection.bluetooth import start_ble_client_thread, BLEClient
from calibration.robotmotor_calibration import calibrate_robot_movement
from calibration.cam_calibrationV2 import collect_calibration_images, load_calibration_parameters, calibrate_camera_from_images
import os
from robotposition.navigation import navigate_to_ball


def main():
    ESP32_ADDRESS = "b0:a7:32:13:a7:26" # Esp MAC address
    CALIBRATION_FILE_PATH = "calibration_parametersV2.npz"
    ble_client = BLEClient(ESP32_ADDRESS)
    ble_thread = start_ble_client_thread(ble_client)  # Start BLE operations in a separate thread and capture the thread object

    stream = livestream.LiveStream()
    mtx, dist = None, None   
    #ret, mtx, dist, tvecs, rvecs = camera_calibration.calibrate_camera(stream) # @AS: we dont need tvecs and rvecs anymore, we are in 2d, removed it
    # if not ret:
    #     print("Camera calibration failed")
    #     return
    live_data = LiveData()
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=5, frame_rate=30, ball_count=8)
    #calibrate_camera_from_images("calibration_images", CALIBRATION_FILE_PATH)
    total_balls = 8
    # Uncomment this line if first time the program runs in the day and calibrate, see the file for instructions
    #calibrate_and_detect_balls(stream, mtx, dist)
    if os.path.exists(CALIBRATION_FILE_PATH):
        mtx, dist, _, _ = load_calibration_parameters(CALIBRATION_FILE_PATH)
        print("Loaded existing calibration parameters.")
    else:
        print("Calibration parameters not found. Please run calibration process.")
        collect_calibration_images(stream, "calibration_images", num_images=40)
        calibrate_camera_from_images("calibration_images", CALIBRATION_FILE_PATH)
    

    # Calibrate pwm for the motors, comment when hardcoded and MCU is flashed again with new calibrated values
    #calibrate_robot_movement(stream, mtx, dist, ble_client)
    while True:
        try:
            frame = stream.get_frame()
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        if frame is None:
            print("Failed to get frame")
            break
        
        frame_undistorted = cv2.undistort(frame, mtx, dist)
        front_point = None  # Reset front_point each iteration to ensure its up-to-date
        closest_ball = None
        # Process ArUco markers
        aruco_corners, aruco_ids, _ = aruco_detection.detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)  # Note: 'frame' is not used after this point
        if aruco_ids is not None and aruco_corners:
            # Get corners from aruco detection to calculate mid vector and direction
            for corner_group in aruco_corners:
                pass
                #frame_undistorted, front_point, rear_point= calculate_and_draw_points(frame_undistorted, corner_group[0])
        
        detected_balls = detect_balls(frame_undistorted, mtx, dist)  # Keep checking for moving objects
        current_time = time.time() 
        ball_confirmation.update_detections(detected_balls, current_time)
        confirmed_balls = ball_confirmation.get_confirmed_balls_positions()
        live_data.update_balls_data(confirmed_balls)

        # Draw detected and confirmed balls
        for ball_pos in detected_balls:
            cv2.circle(frame_undistorted, tuple(ball_pos), 10, (0, 255, 0), 2)
        for confirmed_ball_pos in confirmed_balls:
            cv2.circle(frame_undistorted, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)
            cv2.putText(frame_undistorted, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if front_point is not None and confirmed_balls: # hybrid a* implementation here
            frame_undistorted, closest_ball = find_closest_ball(front_point, confirmed_balls, frame_undistorted, total_balls)
            
        if closest_ball is not None: # hybrid a* implementation here 
            #navigate_to_ball(stream, mtx, dist, ble_client, closest_ball, front_point, rear_point)
            total_balls -= 1
            
            if total_balls == 0:
                print("All balls sucked, heading to goal")
                # logic regarding goal position and 
                break
        # Show the frame
        cv2.imshow('Live Stream', frame_undistorted)
        if cv2.getWindowProperty('Live Stream', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break



    stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
