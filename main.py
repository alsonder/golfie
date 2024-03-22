import cv2
import time
from livestream import livestream
from calibration import camera_calibration # Import the calibration function
from detection import aruco_detection, ball_detection  # Import the detection functions
from livedata.livedata import LiveData  # Import the LiveData class

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

    while True:
        try:
            # Get a frame
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
        live_data.update_aruco_data(aruco_position, aruco_orientation)

        # Detect balls in the frame (live stream)
        balls_position, frame = ball_detection.detect_balls(stream, mtx, dist)
        live_data.update_balls_data(balls_position)

        # Show the frame
        cv2.imshow('Live Stream', frame)

        # Check if the window is open
        if cv2.getWindowProperty('Live Stream', 0) < 0:
            print("Window closed")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break the loop when 'q' is pressed
            break

        # Update live data every second
        time.sleep(1)

    # Release the stream
    stream.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()