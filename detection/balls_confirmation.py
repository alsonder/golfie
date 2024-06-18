import time
import numpy as np

class BallConfirmation:
    def __init__(self, confirmation_threshold=0.18, removal_threshold=0.1, time_window=5, frame_rate=5, ball_count=8):
        self.confirmation_threshold = confirmation_threshold  # Minimum ratio of frames in which a ball must be detected to be confirmed
        self.removal_threshold = removal_threshold  # Minimum ratio of frames without the ball to consider it removed
        self.ball_count = ball_count
        self.time_window = time_window  # Time window in seconds for considering detection data
        self.frame_rate = frame_rate  # Frame rate of the camera
        self.detections = {}  # Temporary storage for new detections
        self.confirmed_balls = {}  # Confirmed balls with last seen time

    def update_detections(self, detected_balls, current_time):
        proximity_threshold = 30  # Distance threshold to consider detections as the same ball

        # Process new detections
        for position in detected_balls:
            ball_id = tuple(position)
            found = False

            # Check against existing confirmed balls
            for confirmed_id in list(self.confirmed_balls.keys()):
                if np.linalg.norm(np.array(ball_id) - np.array(confirmed_id)) < proximity_threshold:
                    self.confirmed_balls[confirmed_id] = current_time
                    found = True
                    break

            # If not found, it might be a new ball
            if not found:
                if ball_id not in self.detections:
                    self.detections[ball_id] = []
                self.detections[ball_id].append(current_time)

        self._cleanup_detections(current_time)
        self._confirm_or_remove_balls(current_time)

    

    def _confirm_or_remove_balls(self, current_time):
        removal_delay = 0.3  # Delay before considering a ball removed
        for ball_id, times in list(self.detections.items()):
            detection_ratio = len(times) / (self.time_window * self.frame_rate)

            # Confirm or remove based on detection ratio and time since last seen
            if detection_ratio >= self.confirmation_threshold:
                self.confirmed_balls[ball_id] = current_time
            if ball_id in self.confirmed_balls:
                time_since_last_seen = current_time - self.confirmed_balls[ball_id]
                if time_since_last_seen >= removal_delay:
                    print(f"Removing ball ID: {ball_id} due to inactivity for {time_since_last_seen} seconds.")
                    del self.confirmed_balls[ball_id]
                    print("Confirmed balls:", self.confirmed_balls)

    def _cleanup_detections(self, current_time):
        # Remove outdated detections
        to_remove = []
        for ball_id, times in list(self.detections.items()):
            self.detections[ball_id] = [t for t in times if current_time - t <= self.time_window]
            if not self.detections[ball_id]:
                to_remove.append(ball_id)

        for ball_id in to_remove:
            del self.detections[ball_id]

    def get_confirmed_balls_positions(self):
        return [tuple(ball_id) for ball_id in self.confirmed_balls.keys()]

