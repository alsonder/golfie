import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt

def midpoint(x1, y1, x2, y2):
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def detect_aruco(frame):
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if ids is not None:
        #aruco.drawDetectedMarkers(frame, corners, ids)
        #print(*corners[0])
        midpoint_aruco = midpoint(
            *midpoint(*corners[0][0][0], *corners[0][0][1]),
            *midpoint(*corners[0][0][2], *corners[0][0][3])
        )    
        print("Aruco Detection Successful")
        return midpoint_aruco
'''
def detect_aruco_test(frame):
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if ids is not None:
        midpoint_aruco = midpoint(
            *midpoint(*corners[0][0][0], *corners[0][0][1]),
            *midpoint(*corners[0][0][2], *corners[0][0][3])
                )
        
        aruco.drawDetectedMarkers(frame, corners, ids)
        print(midpoint_aruco)
    return corners, ids, frame

# Example usage
image_path = 'global_values/test_image.png'
frame = cv2.imread(image_path)
if frame is None:
    print("Failed to load the image")
else:
    aruco_corners, aruco_ids, annotated_frame = detect_aruco_test(frame)
    plt.imshow(cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB))
    plt.title('Detected ArUco markers')
    plt.show()
'''