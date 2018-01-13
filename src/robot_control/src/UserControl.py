#!/usr/bin/python

import numpy as np
np.set_printoptions(threshold=np.nan)
import sys
import select

class UserControl():
    """
    Class used for manually controlling the robot linear and angular velocity
    """
    def __init__(self):
        self.vel = 0
        self.omega = 0

        self.UP = 'w'
        self.DOWN = 's'
        self.LEFT = 'a'
        self.RIGHT = 'd'

        self.timeout = 0.01
        self.read_list = [sys.stdin]
        self.adjust = 0.05        # adjustment factor for steering

        
    def compute_vel(self):
        """
        inputs:
        key_dir: direction of arrow key press. 
          up/down:     increase/decrease linear velocity
          left/right:  increase/decrease angular velocity
        """
        # this needs to be non-blocking
        #key_dir = input("Press wasd to control:")     

        # files monitored for input

        ready = select.select(self.read_list, [], [], self.timeout)[0]
        if not ready:
            pass 
        else:
            file = ready[0]
            keys = file.readline()
            key_dir = keys[0]
            mag = len(keys) - 1  # amount to make adjustment by
            delta = self.adjust*mag
            

            if key_dir is self.UP:
                self.vel += delta
            elif key_dir == self.DOWN:
                self.vel -= delta
            elif key_dir == self.LEFT:
                self.omega += delta
            elif key_dir == self.RIGHT:
                self.omega -= delta

            print('Linear: {0:.2f}\n Angular: {1:.2f} '.format(self.vel, self.omega))

        # if key press is q, then quit entire program
        # modify program to no longer need to press return key after each command
        # enforce min and max on linear and angular velocities    
        
        return self.vel,self.omega,False

