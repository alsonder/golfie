import cv2
import os

cameraid = 1
class Camera:
    def __init__(self, camera_id=cameraid):
        # Initialize the camera with the specified camera ID
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            return

    def capture_image(self, file_path):
        # Capture a single frame from the camera
        ret, frame = self.cap.read()
        if ret:
            # Save the captured image to a file
            cv2.imwrite(file_path, frame)
            print(f"Image saved as {file_path}")
        else:
            print("Error: Failed to capture image.")

    def release(self):
        # Release the camera and close all windows
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Ensure the folder exists
    folder_path = 'global_values'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Initialize camera, capture image, and release resources
    camera = Camera(camera_id=cameraid)  # Change camera_id if necessary
    camera.capture_image(os.path.join(folder_path, 'test_image.png'))
    camera.release()
