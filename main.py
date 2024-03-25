import cv2
import time
from livestream import livestream
from calibration import camera_calibration # Import the calibration function
from detection import aruco_detection  # Import the detection functions
from livedata.livedata import LiveData  # Import the LiveData class
from ball_image_calibration import calibrate_and_detect_balls
from detection.ball_detection import detect_balls
from detection.balls_confirmation import BallConfirmation

def main():
    # Create a LiveStream object
    stream = livestream.LiveStream()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = camera_calibration.calibrate_camera(stream)
    
    if not ret:
        print("Camera calibration failed")
        return

    # Create a LiveData object
    live_data = LiveData()
    
    # Uncomment below line to calibrate the circle detection , comment it when running the program with the settings
    #calibrate_and_detect_balls(stream, mtx, dist)
    
    # Instanciate the BallConfirmation object
    ball_confirmation = BallConfirmation(confirmation_threshold=0.1, removal_threshold=0.8, time_window=5, frame_rate=30)
    while True:
        try:
            # Get a frameq
            frame = stream.get_frame()
        except Exception as e:
            print(f"Error getting frame: {e}")
            break

        # Check if the frame is valid
        if frame is None:
            print("Failed to get frame")
            break

        # Detect ArUco markers in the frame (live stream)
        aruco_position, aruco_orientation, frame = aruco_detection.detect_aruco(stream, mtx, dist, markerLength=0.05)
        print("Aruco position:", aruco_position)
        print("Aruco orientation:", aruco_orientation)
        live_data.update_aruco_data(aruco_position, aruco_orientation)

        # Detect balls in the frame (live stream)   
        detected_balls = detect_balls(frame, mtx, dist)
        current_time = time.time() # adds timestamp for observation
        ball_confirmation.update_detections(detected_balls, current_time) #send it to the confirmation class
        confirmed_balls = ball_confirmation.get_confirmed_balls_positions() #get the list back
        live_data.update_balls_data(confirmed_balls)  # Update LiveData with confirmed balls

        # Draw detected (unconfirmed) and confirmed balls on the fame
        for ball_pos in detected_balls:
            cv2.circle(frame, tuple(ball_pos), 10, (0, 255, 0), 2)  # Green circle for detected balls
        
        for confirmed_ball_pos in confirmed_balls:
            cv2.circle(frame, tuple(confirmed_ball_pos), 10, (0, 0, 255), 2)  # Red circles for confirmed balls (firm balls)
            cv2.putText(frame, f"{confirmed_ball_pos}", tuple(confirmed_ball_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Show the frame
        cv2.imshow('Live Stream', frame)

        # Check if the window is open
        if cv2.getWindowProperty('Live Stream', 0) < 0:
            print("Window closed")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break the loop when 'q' is pressed
            break

        # Update live data every second (Rasmus: its not necessary anymore to sleep it, it runs fine real time)
        #time.sleep(1)

    # Release the stream
    stream.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()