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
        self.balls_position = [] # list of firm balls

    def update_aruco_data(self, position, orientation):
        self.aruco_position = position
        self.aruco_orientation = orientation

    def update_balls_data(self, positions):
        self.balls_position = positions

    def get_balls_position(self):
        return self.balls_position