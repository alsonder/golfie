import numpy as np
import cv2

def find_closest_ball(front_point, confirmed_balls, frame):
    """
    Finds the closest ball to the front point of the robot and draws a line to it.
    
    Parameters:
    - front_point: The front point of the robot in pixels.
    - confirmed_balls: List of confirmed ball positions in pixels.
    - frame: The image frame to draw on.
    
    Returns:
    - Updated frame with the shortest path drawn.
    """
    closest_ball = None
    min_distance = np.inf
    for ball_pos in confirmed_balls:
        distance = np.linalg.norm(np.array(front_point) - np.array(ball_pos))
        if distance < min_distance:
            min_distance = distance
            closest_ball = ball_pos

    # Draw the line to the closest ball if one is found
    if closest_ball is not None:
        cv2.line(frame, front_point, tuple(closest_ball), (255, 0, 0), 2)  # Draw a blue line

    return frame

def draw_path(frame, start_point, end_point):
    """Draws a line representing the path from the robot to the ball.
    
    Args:
        frame (np.array): The current frame to draw on.
        start_point (tuple): Starting point coordinates (robot).
        end_point (tuple): Ending point coordinates (ball).
    """
    cv2.line(frame, start_point, end_point, (255, 255, 0), 2)  # Yellow line

