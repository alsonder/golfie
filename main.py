import cv2
import time
import os
import threading
import sys
import numpy as np
from livestream import livestream
from calibration import camera_calibration
from detection import aruco_detection
from livedata.livedata import LiveData
from detection.ball_detection import detect_balls
from detection.balls_confirmation import BallConfirmation
from pathfinding.robot_point_calculation import calculate_and_draw_points
from pathfinding.shortest_path import find_closest_ball
from ball_image_calibration import calibrate_and_detect_balls
#from connection.bluetooth import start_ble_client_thread, BLEClient
###
from global_values.find_goal import decide_goal_loc
from global_values.find_aruco import detect_aruco
from global_values.find_cross import find_and_draw_red_cross
from global_values.find_egg import detect_egg
from global_values.find_walls import get_line_pixels_and_corners
from global_values.create_path import nearest_neighbor, a_star_search, nearest_neighbor_simplified
from global_values.create_grid import gridCreation
from global_values.display_grid import visualize_grid
from global_values.create_path import nearest_neighbor, a_star_search
from global_values.create_path import calculate_distance_to_wall
###
from calibration.robotmotor_calibration import calibrate_robot_movement
from calibration.cam_calibrationV2 import collect_calibration_images, load_calibration_parameters, calibrate_camera_from_images
from robotposition.navigation import navigate_to_ball

#from global_values import all_values

def main():
    ESP32_ADDRESS = "30:c9:22:11:e9:92"  # Esp MAC address
    CALIBRATION_FILE_PATH = "calibration_parametersV2.npz"
#   ble_client = BLEClient(ESP32_ADDRESS)
#   ble_thread = start_ble_client_thread(ble_client)  # Start BLE operations in a separate thread and capture the thread object
    
    ########################################
    ### --- START OF INITIAL TESTING --- ###
    ########################################
    
    starter_cap = cv2.VideoCapture(2)
    if not starter_cap.isOpened():
        print("Cannot open camera")
        return None
    success = 0
    while (success < 9):
        success = 0
        ret, starter_frame = starter_cap.read()
        ROW, COL, channels = starter_frame.shape
        print("Image : Width:", COL, "Height:", ROW)
        cv2.imwrite('starter_image.png', starter_frame)

        #global functions
    
        try: aruco_location = detect_aruco(starter_frame); success+=1
        except: print("Aruco Detection         | Failed")
        try: find_cross = find_and_draw_red_cross(starter_frame); success+=1
        except: print("Cross Detection         | Failed")
        try: egg_loc = detect_egg(starter_frame); success+=1
        except: print("Egg Detection           | Failed")
        try: wall_corner_locations, line_pixels = get_line_pixels_and_corners(starter_frame); success+=1
        except: print("Wall Detection          | Failed"); print("Corner Identification   | Successful")

        try: goal_location = decide_goal_loc(aruco_location,wall_corner_locations); success+=1
        except: print("Goal Detection          | Failed")
        try: grid, weightedGrid = gridCreation(ROW+1, COL, wall_corner_locations+find_cross+egg_loc); success+=1
        except: print("Grid Creation           | Failed"); print("Weights Added to Grid   | Failed")

        try: goal = (round(goal_location[0][1]/2),round(goal_location[0][0]/2)); success+=1
        except: pass
        try: aruco = (aruco_location[1],aruco_location[0]); success+=1
        except: pass
        try: path = a_star_search(grid, aruco, goal, weightedGrid); success+=1
        except: print("Path To Node Found      | Failed")
        print("Successes : ", success)
        if (success < 9):
            time.sleep(4.5)
            print("------------------------------------------------------")
    
    starter_cap.release()

    print("\n - - - Some values are - - -")
    print("aruco_loc = ", aruco_location)
    print("find_cross = ",find_cross[:4]) #4 first values of findcross
    print("egg_loc = ", egg_loc[:4]) #4 first values of egg
    print("line_pixels = ", line_pixels[:4]) #4 first values of wall
    print("goal_loc = ", goal_location)
    visualize_grid(grid, aruco, [aruco,goal], path)

    transposed_matrix = []
    for col in range(len(grid[0])):
        transposed_row = []
        for row in range(len(grid)):
            transposed_row.append(grid[row][col])
        transposed_matrix.append(transposed_row)
    grid = transposed_matrix
    print(len(grid), len(grid[0]))

    ########################################
    ### --- END OF INITIAL TESTING --- ###
    ########################################
    stream = livestream.LiveStream()
    mtx = np.array([[427.37649845,   0.,         325.09842022],
    [  0.,         400.30539872, 242.34431018],
    [  0.,           0.,           1.        ]])

    dist = np.array([[0, 0,0,  0, 0]]) 
    live_data = LiveData()
    total_balls = 1
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=5, frame_rate=30, ball_count=total_balls)

    if os.path.exists(CALIBRATION_FILE_PATH):
        mtx, dist, _, _ = load_calibration_parameters(CALIBRATION_FILE_PATH)
        print("Loaded existing calibration parameters.")
    else:
        print("Calibration parameters not found. Please run calibration process.")
        collect_calibration_images(stream, "calibration_images", num_images=40)
        calibrate_camera_from_images("calibration_images", CALIBRATION_FILE_PATH)
        previousOrderOfPoints = 0
        orderOfPoints = None
        path = None
        oldAruco = None
        count = 0
        is_navigating = False

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
        front_point, rear_point, closest_ball = None, None, None

        # Process ArUco markers
        aruco_corners, aruco_ids, _ = aruco_detection.detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)
        if aruco_ids is not None and aruco_corners:
            for corner_group in aruco_corners:
                frame_undistorted, front_point, rear_point = calculate_and_draw_points(frame_undistorted, corner_group[0])

        detected_balls = detect_balls(frame_undistorted, mtx, dist)
        current_time = time.time()
        ball_confirmation.update_detections(detected_balls, current_time)
        confirmed_balls = ball_confirmation.get_confirmed_balls_positions()
        live_data.update_balls_data(confirmed_balls)
        print("-----------------------------------")
        print("Confirmed balls: ", confirmed_balls)
        print("Number of confirmed balls: ", len(confirmed_balls))

        #print("Confirmed balls:", confirmed_balls)


        # Draw detected and confirmed balls
        for ball_pos in detected_balls:
            cv2.circle(frame_undistorted, tuple(ball_pos), 10, (0, 255, 0), 2)
        for confirmed_ball_pos in confirmed_balls:
            cv2.circle(frame_undistorted, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)
            cv2.putText(frame_undistorted, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        


        if confirmed_balls is not None and len(confirmed_balls) >= 2 and len(confirmed_balls) != previousOrderOfPoints and front_point is not None:
            orderOfPoints = nearest_neighbor_simplified([front_point] + confirmed_balls)
            previousOrderOfPoints = len(confirmed_balls)
            path = a_star_search(grid, front_point, orderOfPoints[0], weightedGrid)
        
        if(oldAruco != front_point and confirmed_balls is not None and len(confirmed_balls) >= 2) and front_point is not None:
            oldAruco = front_point
            orderOfPoints = nearest_neighbor_simplified([front_point] + confirmed_balls)
            path = a_star_search(grid, front_point, orderOfPoints[0], weightedGrid)

        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 0:
                    cv2.circle(frame_undistorted, [i,j], 1, (0, 255, 255), 2)    


        if(path is not None):
            for coordinate in path: 
                cv2.circle(frame_undistorted, coordinate, 1, (255, 255, 255), 2)    

        if (count >= 100):
            count = 0
            print("her")
            print(orderOfPoints)
            print("her")
        else: 
            count+=1


        #if front_point is not None and confirmed_balls:
            #frame_undistorted, closest_ball = find_closest_ball(front_point, confirmed_balls, frame_undistorted, total_balls)

        if closest_ball is not None and not is_navigating:
            print("Starting navigation.")
            is_navigating = True
            #navigate_to_ball(stream, mtx, dist, ble_client, closest_ball, front_point, rear_point, lambda: None)
            is_navigating = False

        cv2.imshow('Live Stream', frame_undistorted)

        if cv2.getWindowProperty('Live Stream', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
