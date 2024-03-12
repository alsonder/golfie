# main.py
import argparse
from calibration.camera_calibration import calibrate_camera
from detection.wall_detection import detect_walls

def main():
    parser = argparse.ArgumentParser(description='Robot control system.')
    parser.add_argument('command', choices=['calibrate'], help='The command to execute.')

    args = parser.parse_args()

    if args.command == 'calibrate':
        success = calibrate_camera()
        if success:
            detect_walls()

if __name__ == "__main__":
    main()