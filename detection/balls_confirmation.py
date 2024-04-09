import time
import numpy as np

class BallConfirmation:
    # pass in variables for confirmation, 20% of the frames must contain tthe circle for it to become a firm ball, to be a removed ball 80% of 
    #frames must be without the ball, if its been still for 5 seconds, it becomes a firm ball, usage in a camera which processes 30 fps
    def __init__(self, confirmation_threshold=0.18, removal_threshold=0.8, time_window=5, frame_rate=30):
        self.confirmation_threshold = confirmation_threshold
        self.removal_threshold = removal_threshold
        self.time_window = time_window
        self.frame_rate = frame_rate
        self.detections = {}
        self.confirmed_balls = {}
    
    # Updates the class with the latest detections, and a time, each detected ball is identified by its position, which is tracked and evaluated

    def update_detections(self, detected_balls, current_time):
        proximity_threshold = 15  # Define how close detections need to be (in pixels) to be considered the same ball

        for position in detected_balls:
            ball_id = tuple(position)
            # Check for proximity to existing confirmed balls
            close_to_confirmed = None
            for confirmed_id in self.confirmed_balls.keys():
                if np.linalg.norm(np.array(ball_id) - np.array(confirmed_id)) < proximity_threshold:
                    close_to_confirmed = confirmed_id
                    break

            if close_to_confirmed:
                # Update timestamp for closely matched confirmed ball
                self.confirmed_balls[close_to_confirmed] = current_time
            else:
                if ball_id not in self.detections:
                    self.detections[ball_id] = []
                self.detections[ball_id].append(current_time)

        self._cleanup_detections(current_time)
        self._confirm_or_remove_balls(current_time)


    # removes old balls (!).. which is detections older than the time_window formm the detections dict so we dont fill the array with outdated positions
    def _cleanup_detections(self, current_time):
        to_remove = []
        for ball_id, times in self.detections.items():
            self.detections[ball_id] = [t for t in times if current_time - t <= self.time_window]
            if not self.detections[ball_id]:
                to_remove.append(ball_id)
        for ball_id in to_remove:
            del self.detections[ball_id]

    # This check if its a new ball, or it should remove previously firm balls
    def _confirm_or_remove_balls(self, current_time):
        removal_delay = 3  # Additional delay in seconds before removing a ball to account for detection inconsistency
        for ball_id, times in list(self.detections.items()):
            detection_ratio = len(times) / (self.frame_rate * self.time_window)
            
            if detection_ratio >= self.confirmation_threshold:
                self.confirmed_balls[ball_id] = times[-1]
            elif ball_id in self.confirmed_balls:
                time_since_last_seen = current_time - self.confirmed_balls[ball_id]
                absence_ratio = time_since_last_seen / self.frame_rate
                if absence_ratio > removal_delay:  # Use a fixed delay to determine removal, adjusting for robustness
                    del self.confirmed_balls[ball_id]


    # Returns positions of firm balls
    def get_confirmed_balls_positions(self):
        return [list(ball_id) for ball_id in self.confirmed_balls.keys()]
