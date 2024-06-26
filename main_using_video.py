import cv2
import time
import sys
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
import os
from robotposition.navigation import navigate_to_ball

#from global_values import all_values

def main():
    ESP32_ADDRESS = "b0:a7:32:13:a7:26" # Esp MAC address
    CALIBRATION_FILE_PATH = "calibration_parametersV2.npz"
#   ble_client = BLEClient(ESP32_ADDRESS)
#   ble_thread = start_ble_client_thread(ble_client)  # Start BLE operations in a separate thread and capture the thread object
    
    ########################################
    ### --- START OF INITIAL TESTING --- ### 
    ########################################
    
    video_path = "video_files/ball_movement.mp4"
    starter_cap = cv2.VideoCapture(video_path)
    if not starter_cap.isOpened():
        print("Cannot open camera")
        return None
    success = 0
    while (success < 1):
        success = 0
        ret, starter_frame = starter_cap.read()
        ROW, COL, channels = starter_frame.shape
        print("Image : Width:", ROW, "Height:", COL)
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
        try: grid, weightedGrid = gridCreation(ROW+50,COL+50, wall_corner_locations+find_cross+egg_loc); success+=1
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
        goal_location = [(10,20),(20,10),(30,30),(40,40)]
    starter_cap.release()

    print("\n - - - Some values are - - -")
    print("aruco_loc = ", aruco_location)
    print("find_cross = ",find_cross[:4]) #4 first values of findcross
    print("egg_loc = ", egg_loc[:4]) #4 first values of egg
    print("line_pixels = ", line_pixels[:4]) #4 first values of wall
    print("goal_loc = ", goal_location)
    #visualize_grid(grid, aruco, [aruco,goal], path)
    
    ######################################
    ### --- END OF INITIAL TESTING --- ###
    ######################################


    ######################################
    ### ---     START OF VIDEO     --- ###
    ######################################


    #stream = livestream.LiveStream()
    # replaced for video : 
    video_path = "video_files/ball_movement.mp4"
    video_cap = cv2.VideoCapture(video_path)
    if not video_cap.isOpened():
        print("Error: Cannot open video file.")
        return

    mtx, dist = None, None   
    #ret, mtx, dist, tvecs, rvecs = camera_calibration.calibrate_camera(stream) # @AS: we dont need tvecs and rvecs anymore, we are in 2d, removed it
    # if not ret:
    #     print("Camera calibration failed")
    #     return
    live_data = LiveData()
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=10, frame_rate=30, ball_count=8)
    
    #calibrate_camera_from_images("calibration_images", CALIBRATION_FILE_PATH)
    total_balls = 8
    # Uncomment this line if first time the program runs in the day and calibrate, see the file for instructions
    #calibrate_and_detect_balls(stream, mtx, dist)
    if os.path.exists(CALIBRATION_FILE_PATH):
        mtx, dist, _, _ = load_calibration_parameters(CALIBRATION_FILE_PATH)
        print("Loaded existing calibration parameters.")
    else:
        print("Calibration parameters not found. Please run calibration process.")
        collect_calibration_images(video_cap, "calibration_images", num_images=40) #video_cap was stream before video
        calibrate_camera_from_images("calibration_images", CALIBRATION_FILE_PATH)
    

    previousOrderOfPoints = 0
    orderOfPoints = None
    path = None
    currentcorner = None

    # Calibrate pwm for the motors, comment when hardcoded and MCU is flashed again with new calibrated values
    #calibrate_robot_movement(stream, mtx, dist, ble_client)
    
    #replaced for video
    '''while True:
        try:
            ret, frame =  video_cap.read()
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        if frame is None:
            print("Failed to get frame")
            break'''
    

    
    while True:
        ret, frame = video_cap.read()

        
        # If frame is read successfully
        if ret:
            # Display the frame
            cv2.imshow('Video Loop', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Wait for the 'q' key to quit
                break
        else:
            # If the end of the video is reached, reset to the first frame
            video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        
        frame_undistorted = cv2.undistort(frame, mtx, dist)
        front_point = None  # Reset front_point each iteration to ensure its up-to-date
        closest_ball = None
        # Process ArUco markers
        aruco_corners, aruco_ids, _ = aruco_detection.detect_aruco(frame_undistorted, mtx, dist, markerLength=0.08)  # Note: 'frame' is not used after this point
        if aruco_ids is not None and aruco_corners:
            # Get corners from aruco detection to calculate mid vector and direction
            for corner_group in aruco_corners:
                frame_undistorted, front_point, rear_point= calculate_and_draw_points(frame_undistorted, corner_group[0])
        
        detected_balls = detect_balls(frame_undistorted, mtx, dist)  # Keep checking for moving objects
        current_time = time.time() 
        ball_confirmation.update_detections(detected_balls, current_time)
        confirmed_balls = ball_confirmation.get_confirmed_balls_positions()
        live_data.update_balls_data(confirmed_balls)

        # Draw detected and confirmed balls
        for ball_pos in detected_balls:
            cv2.circle(frame_undistorted, tuple(ball_pos), 10, (0, 255, 0), 2)
        for confirmed_ball_pos in confirmed_balls:
            #replaced for video
            #cv2.circle(frame_undistorted, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)
            #cv2.putText(frame_undistorted, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(frame_undistorted, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)
            cv2.putText(frame_undistorted, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        


        if confirmed_balls is not None and len(confirmed_balls) > 2 and len(confirmed_balls) != previousOrderOfPoints:
            orderOfPoints = nearest_neighbor_simplified(front_point + confirmed_balls)
            previousOrderOfPoints = len(confirmed_balls)
            path = a_star_search(grid, front_point, orderOfPoints[0], weightedGrid)
        
        if(path is not None):
            for coordinate in path: 
                cv2.circle(frame_undistorted, (coordinate[0], coordinate[1]), 1, (255, 255, 255), 2)    


        #if front_point is not None and confirmed_balls: # hybrid a* implementation here
            #frame_undistorted, closest_ball = find_closest_ball(front_point, confirmed_balls, frame_undistorted, total_balls)
            
        if closest_ball is not None: # hybrid a* implementation here 
            #navigate_to_ball(stream, mtx, dist, ble_client, closest_ball, front_point, rear_point)
            total_balls -= 1
            
            if total_balls == 0:
                print("All balls sucked, heading to goal")
                # logic regarding goal position and 
                break
        # Display the processed frame
        cv2.imshow('Processed Video', frame) #frame/frame_undistorted
        if cv2.getWindowProperty('Processed Video', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Show the frame
        cv2.imshow('Live Stream', frame_undistorted)
        if cv2.getWindowProperty('Live Stream', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break



    video_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
