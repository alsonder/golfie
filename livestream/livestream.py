import cv2

class LiveStream:
    def __init__(self, camera_id=2):  # Change this line
        self.cap = cv2.VideoCapture(camera_id)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Could not read frame from camera")
        return frame

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def show_frame(self, frame):
        cv2.imshow('Live Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
        return True