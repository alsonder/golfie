import cv2
import numpy as np
import os
from glob import glob

# Configuration
CHECKERBOARD = (9, 6)
CALIBRATION_IMAGES_DIRECTORY = "calibration_images"
CALIBRATION_FILE_PATH = "calibration_parametersV2.npz"
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
SQUARE_SIZE = 2.5  # The actual size of the checkerboard square in your unit of choice

def collect_calibration_images(stream, save_directory, num_images=40):
    """
    Captures calibration images from the provided video stream, applies histogram equalization,
    and saves them to the specified directory.
    """
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)
    print(f"Starting to collect {num_images} calibration images. Press 'c' to capture.")
    count = 0
    while count < num_images:
        frame = stream.get_frame()

        # Preprocessing: Histogram Equalization
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = apply_clahe_to_gray_image(gray)
        cv2.imshow('Live Stream (Preprocessed)', gray)  # Show preprocessed image
        if cv2.waitKey(1) & 0xFF == ord('c'):
            filepath = os.path.join(save_directory, f"calibration_{count}.jpg")
            cv2.imwrite(filepath, gray)  # Save the preprocessed image
            print(f"Saved {filepath}")
            count += 1
    print("Completed collecting calibration images.")

def calibrate_camera_from_images(image_directory, save_file_path):
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2) * SQUARE_SIZE

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob(os.path.join(image_directory, '*.jpg'))
    used_images_count = 0

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = apply_clahe_to_gray_image(gray)  # Apply CLAHE before corner detection        # Enhanced corner detection flags
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH) 
        if ret == True:
            used_images_count += 1
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, winSize=(11,11), zeroZone=(-1,-1), criteria=CRITERIA)
            imgpoints.append(corners2)

            # Draw and display the corners for user verification
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)  # give enough time to see the image

    print(f"Total usable images: {used_images_count} of {len(images)}")

    if len(objpoints) > 0 and len(imgpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        if ret:
            np.savez(save_file_path, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
            print(f"Calibration successful, parameters saved to {save_file_path}.")
        else:
            print("Calibration failed.")
    else:
        print("Not enough valid images for calibration.")
def load_calibration_parameters(file_path):
    """
    Loads saved calibration parameters.
    """
    with np.load(file_path) as data:
        mtx = data['mtx']
        dist = data['dist']
        rvecs = data['rvecs']
        tvecs = data['tvecs']
    print("Calibration parameters loaded.")
    return mtx, dist, rvecs, tvecs

def apply_clahe_to_gray_image(gray_image):
    """
    Applies CLAHE to a grayscale image to improve contrast adaptively.
    """
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(gray_image)
