from livestream import livestream
from pathfinding import grid_management
from calibration import camera_calibration
import cv2

def main():
    # Create a LiveStream object
    stream = livestream.LiveStream()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = camera_calibration.calibrate_camera(stream)

    if not ret:
        print("Camera calibration failed")
        return

    while True:
        # Get a frame
        frame = stream.get_frame()

        # Check if the frame is valid
        if frame is None:
            print("Failed to get frame")
            break

        # Undistort the frame
        frame = cv2.undistort(frame, mtx, dist, None, mtx)

        # Draw a grid on the frame
        frame = grid_management.draw_grid(frame, 8)

        # Show the frame
        cv2.imshow('Live Stream', frame)

        # Check if the window is open
        if cv2.getWindowProperty('Live Stream', 0) < 0:
            print("Window closed")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break the loop when 'q' is pressed
            break

    # Release the stream
    stream.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()