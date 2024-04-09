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

def main():
    stream = livestream.LiveStream()
    ret, mtx, dist, tvecs, rvecs = camera_calibration.calibrate_camera(stream) # @AS: we dont need tvecs and rvecs anymore, we are in 2d, removed it
    if not ret:
        print("Camera calibration failed")
        return

    live_data = LiveData()
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=5, frame_rate=30)
    #calibrate_and_detect_balls(stream, mtx, dist)
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
        front_point = None  # Reset front_point each iteration to ensure it's up-to-date

        # Process ArUco markers
        aruco_corners, aruco_ids, _ = aruco_detection.detect_aruco(stream, mtx, dist, markerLength=0.08)  # Note: 'frame' is not used after this point
        if aruco_ids is not None and aruco_corners:
            for corner_group in aruco_corners:
                frame_undistorted, front_point = calculate_and_draw_points(frame_undistorted, corner_group[0])
        
        detected_balls, orange_ball = detect_balls(frame_undistorted, mtx, dist)  
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

        if front_point is not None and confirmed_balls:
            frame_undistorted = find_closest_ball(front_point, confirmed_balls, frame_undistorted)

        # Show the frame
        cv2.imshow('Live Stream', frame_undistorted)
        if cv2.getWindowProperty('Live Stream', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break



    stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
