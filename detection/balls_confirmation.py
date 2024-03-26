import time

class BallConfirmation:
    # pass in variables for confirmation, 20% of the frames must contain tthe circle for it to become a firm ball, to be a removed ball 80% of 
    #frames must be without the ball, if its been still for 5 seconds, it becomes a firm ball, usage in a camera which processes 30 fps
    def __init__(self, confirmation_threshold=0.2, removal_threshold=0.8, time_window=5, frame_rate=30):
        self.confirmation_threshold = confirmation_threshold
        self.removal_threshold = removal_threshold
        self.time_window = time_window
        self.frame_rate = frame_rate
        self.detections = {}
        self.confirmed_balls = {}
    
    # Updates the class with the latest detections, and a time, each detected ball is identified by its position, which is tracked and evaluated

    def update_detections(self, detected_balls, current_time):
        for position in detected_balls:
            ball_id = tuple(position)  
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
        confirmed_ids = list(self.confirmed_balls.keys())
        for ball_id, times in self.detections.items():
            if len(times) / (self.frame_rate * self.time_window) >= self.confirmation_threshold:
                self.confirmed_balls[ball_id] = times[-1]  
            elif ball_id in confirmed_ids and current_time - self.confirmed_balls[ball_id] > (1 - self.removal_threshold) * self.time_window:
                del self.confirmed_balls[ball_id]

    # Returns positions of firm balls
    def get_confirmed_balls_positions(self):
        return [list(ball_id) for ball_id in self.confirmed_balls.keys()]
