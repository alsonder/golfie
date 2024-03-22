"""
This module contains the LiveData class which is used to store 
and update live data from the ArUco marker detection and ball 
detection processes.

Classes:
    LiveData: Stores and updates live data from the ArUco 
    marker detection and ball detection processes.
"""

class LiveData:
    def __init__(self):
        self.aruco_position = None
        self.aruco_orientation = None
        self.balls_position = None

    def update_aruco_data(self, position, orientation):
        self.aruco_position = position
        self.aruco_orientation = orientation

    def update_balls_data(self, position):
        self.balls_position = position