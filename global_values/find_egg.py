import cv2
import numpy as np
from PIL import Image

"""This function will detect a circle on the thick part of the egg. Then calculate points on a circle periphery,
that encapsulates the egg.The standard amount of points made is 360, because it increments 1 degree / point.
The function then returns a numpy array with touples containing coordinates. Example: [(150,300),(145,315)]"""


def detect_egg(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    # Apply the CLAHE filter to the grayscale image
    clahe_gray = clahe.apply(gray)
    # Change below according to the best setting found during calibrate_and_detect_balls()
    lower_hsv = np.array([0, 0, 0])
    upper_hsv = np.array([179, 255, 255])
    gaussian_blur = 1
    param1 = 50
    param2 = 13
    min_radius = 9
    max_radius = 12

    # apply mask which circle detection operates
    blurred_gray = cv2.GaussianBlur(clahe_gray, (gaussian_blur, gaussian_blur), 0)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.dilate(mask, None, iterations=1)

    circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, 1, 1,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)
    egg_list = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            egg_list.append([i[0], i[1]])

    egg_circle = []
    for egg in egg_list:
        for i in range(360):
            X = int(egg[0] + (20 * np.cos(360/(i+1))))
            Y = int(egg[1] + (20 * np.sin(360/(i+1))))
            egg_circle.append((X,Y))
            cv2.circle(frame, (int(X), int(Y)), 1, (0, 255, 0), -1)
            
    
    if egg_circle == None:
        print("Egg not found")
    else:
        print("Egg Detection Successful")       


    return egg_circle  # Returns the 360points created just above

# Load the image
#image_path = "global_values/test_image.png"
#frame = cv2.imread(image_path)

# Detect balls
#detected_egg = detect_egg(frame)
            
        
# Convert the OpenCV image to PIL image
#pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

# Display the PIL image
#pil_image.show()




