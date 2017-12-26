#!/usr/bin/python

import numpy as np
import sys

class UserControl():
    """
    Class used for manually controlling the robot linear and angular velocity
    """
    def __init__(self):
        self.vel = 0
        self.omega = 0

        self.UP = 0
        self.DOWN = 1
        self.LEFT = 2
        self.RIGHT = 3

        
    def compute_vel(self):
        """
        inputs:
        key_dir: direction of arrow key press. 
          up/down:     increase/decrease linear velocity
          left/right:  increase/decrease angular velocity
        """
        key_dir = input("Press arrow key.")                

        if key_dir == self.UP:
            self.vel += 0.1
        elif key_dir == self.DOWN:
            self.vel -= 0.1
        elif key_dir == self.LEFT:
            self.omega += 0.1
        elif key_dir == self.RIGHT:
            self.omega -= 0.1
        
        return self.vel,self.omega

