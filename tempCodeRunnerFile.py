
import cv2
import time
from livestream import livestream
from calibration import camera_calibration
from detection import aruco_detection
from livedata.livedata import LiveData
from detection.ball_detection import detect_balls
from detection.balls_confirmation import BallConfirmation
from pathfinding.robot_point_calculation import calculate_and_draw_points
from pathfinding.shortest_path import find_closest_ball
from ball_image_calibration import calibrate_and_detect_balls
from connection.bluetooth import start_ble_client_thread, BLEClient
from calibration.robotmotor_calibration import calibrate_robot_movement
from calibration.cam_calibrationV2 import collect_calibration_images, load_calibration_parameters, calibrate_camera_from_images
import os
from robotposition.navigation import navigate_to_ball
import detection.egg_detection



def test():

    mtx, dist = None, None

    detection.detect_egg("images_with_eggs.jpg", mtx, dist)
