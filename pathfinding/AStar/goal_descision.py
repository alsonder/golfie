import numpy as np

def get_aruco_center(aruco_corners): #requires touple
    aruco_centroids = []

    for corners in aruco_corners:
        corners_array = np.array(corners)
        centroid_x = np.mean(corners_array[:, 0, 0])
        centroid_y = np.mean(corners_array[:, 0, 1])
        aruco_centroids.append((centroid_x, centroid_y))

    return aruco_centroids #example return : [(x1,y1)]

def get_corners(): #requires touple

    return 0


def decide_goal_loc(aruco_corners, ):
    x1, y1 = get_aruco_center(aruco_corners)
    x2, y2 = get_aruco_center(aruco_corners)
    x3, y3 = get_aruco_center(aruco_corners)
    print("built final coordinates to exit")
    return [(x1, y1),(x2,y2),(x3,y3)] #example return : [(x1,y1),(x2,y2),(x3,y3)]
