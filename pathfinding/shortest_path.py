import cv2
import numpy as np

def find_closest_ball(front_point, confirmed_balls, frame, total_balls):
    closest_ball = None
    if len(confirmed_balls) >= total_balls:
        min_distance = np.inf
        for ball_pos in confirmed_balls:
            distance = np.linalg.norm(np.array(front_point) - np.array(ball_pos))
            if distance < min_distance:
                min_distance = distance
                closest_ball = ball_pos

        if closest_ball is not None:
            #cv2.line(frame, front_point, tuple(closest_ball), (255, 0, 0), 2)  # Draw a blue line for the path
            print("Target ball count reached. Closest ball identified.")
    else:
        print(f"Waiting for all balls to be detected... {len(confirmed_balls)}/{total_balls} detected so far.")

    return frame, closest_ball
