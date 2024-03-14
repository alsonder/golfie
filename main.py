import cv2
from livestream import livestream
from calibration import camera_calibration  # Import the calibration function

def main():
    # Create a LiveStream object
    stream = livestream.LiveStream()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = camera_calibration.calibrate_camera(stream)
    if not ret:
        print("Camera calibration failed")
        return

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

        # Show the frame
        #cv2.imshow('Live Stream', frame)

        # Check if the window is open
        #if cv2.getWindowProperty('Live Stream', 0) < 0:
        #    print("Window closed")
        #    break

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break the loop when 'q' is pressed
            break

    # Release the stream
    stream.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()