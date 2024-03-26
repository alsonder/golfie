import cv2
import time
from livestream import livestream
from calibration import camera_calibration
from detection import aruco_detection
from livedata.livedata import LiveData
from detection.ball_detection import detect_balls
from detection.balls_confirmation import BallConfirmation
from pathfinding.robot_point_calculation import calculate_and_draw_points

def main():
    stream = livestream.LiveStream()
    ret, mtx, dist = camera_calibration.calibrate_camera(stream) # @AS: we dont need tvecs and rvecs anymore, we are in 2d, removed it
    if not ret:
        print("Camera calibration failed")
        return

    live_data = LiveData()
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=5, frame_rate=30)

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

        # Now directly working with 2D corner points of ArUco markers
        aruco_corners, aruco_ids, frame = aruco_detection.detect_aruco(stream, mtx, dist, markerLength=0.08)
        # Update aruco data if needed here using 'aruco_corners' and 'aruco_ids'
        if aruco_ids is not None:
            for corner_group in aruco_corners:
                frame = calculate_and_draw_points(frame_undistorted, corner_group[0]) # Gets the front mid point of the robot
            
        
        detected_balls = detect_balls(frame, mtx, dist) # Get balls!
        current_time = time.time()
        ball_confirmation.update_detections(detected_balls, current_time) # Detect all balls 
        confirmed_balls = ball_confirmation.get_confirmed_balls_positions() # Convert unfirm balls to firm balls
        live_data.update_balls_data(confirmed_balls)

        # Draw detected and firm balls on the frame
        for ball_pos in detected_balls:
            cv2.circle(frame_undistorted, tuple(ball_pos), 10, (0, 255, 0), 2) # Green circle for unfirm balls
        for confirmed_ball_pos in confirmed_balls:
            # Red circle and coordinates for firm balls
            cv2.circle(frame_undistorted, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)
            cv2.putText(frame_undistorted, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Show the frame
        cv2.imshow('Live Stream', frame_undistorted)
        if cv2.getWindowProperty('Live Stream', 0) < 0 or cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
