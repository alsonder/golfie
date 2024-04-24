import cv2
import time
import matplotlib.pyplot as plt
from livestream import livestream
from calibration import camera_calibration
from detection import aruco_detection
from livedata.livedata import LiveData
from detection.ball_detection import detect_balls
from detection.egg_detection import detect_egg
from detection.balls_confirmation import BallConfirmation
from pathfinding.robot_point_calculation import calculate_and_draw_points
from pathfinding.shortest_path import find_closest_ball
from calibration.robotmotor_calibration import calibrate_robot_movement
from calibration.cam_calibrationV2 import collect_calibration_images, load_calibration_parameters, \
    calibrate_camera_from_images
import os
from robotposition.navigation import navigate_to_ball


mtx = None
dist = None
frame = cv2.imread('../images_w_egg/image_0.png')

ball_list = detect_balls(frame, mtx, dist)

print("hello")

print("Hello")
# Display the result
