#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=1
        self.ka=5
        self.kb=0
        self.tol = .1
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        done = False
        theta = state[2]
        deltaX = goal[0] - state[0]
        deltaY = goal[1] - state[1]
        rho = np.sqrt(deltaX**2 + deltaY**2)
        alpha = -theta + np.arctan2(deltaY,deltaX)
        beta = -theta - alpha

        v = self.kp * rho
        omega = self.ka*alpha + self.kb*beta

        if np.abs(rho) < self.tol:
            done = True
        return v,omega,done
