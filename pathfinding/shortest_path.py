import numpy as np
import cv2

def find_closest_ball(front_point, confirmed_balls, frame, ball_count=8):
    """
    Finds the closest ball to the front point of the robot and draws a line to it, only if the target ball count has been reached.
    
    Parameters:
    - front_point: The front point of the robot in pixels.
    - confirmed_balls: List of confirmed ball positions in pixels.
    - frame: The image frame to draw on.
    - ball_count: The target number of balls to be detected before moving.
    
    Returns:
    - Updated frame with the shortest path drawn, if target ball count is reached.
    - Coordinates of the closest ball (None if the target ball count is not reached).
    """
    closest_ball = None
    if len(confirmed_balls) >= ball_count:
        min_distance = np.inf
        for ball_pos in confirmed_balls:
            distance = np.linalg.norm(np.array(front_point) - np.array(ball_pos))
            if distance < min_distance:
                min_distance = distance
                closest_ball = ball_pos

        # Draw the line to the closest ball if one is found
        if closest_ball is not None:
            cv2.line(frame, front_point, tuple(closest_ball), (255, 0, 0), 2)  # Draw a blue line for the path
            print("Target ball count reached. Closest ball identified.")
    else:
        print(f"Waiting for all balls to be detected... {len(confirmed_balls)}/{ball_count} detected so far.")

    return frame, closest_ball

def draw_path(frame, start_point, end_point):
    """Draws a line representing the path from the robot to the ball.
    
    Args:
        frame (np.array): The current frame to draw on.
        start_point (tuple): Starting point coordinates (robot).
        end_point (tuple): Ending point coordinates (ball).
    """
    cv2.line(frame, start_point, end_point, (255, 255, 0), 2)  # Yellow line

