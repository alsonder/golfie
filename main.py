from livestream import livestream
from pathfinding import grid_management
from calibration import camera_calibration
import cv2

def main():
    # Create a LiveStream object
    stream = livestream.LiveStream()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()

    if not ret:
        print("Camera calibration failed")
        return

    while True:
        # Get a frame
        frame = stream.get_frame()

        # Undistort the frame
        frame = cv2.undistort(frame, mtx, dist, None, mtx)

        # Draw a grid on the frame
        frame = grid_management.draw_grid(frame, 8)

        # Show the frame
        if not stream.show_frame(frame):
            break

    # Release the stream
    stream.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()