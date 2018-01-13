#!/usr/bin/python

import numpy as np
import pdb

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, min_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp = 0.2
        self.ka = 2
        self.kb = 0
        self.tol = .05
        self.MAX_SPEED = max_speed
        self.MIN_SPEED = min_speed
        self.MAX_OMEGA = max_omega
        self.done = False

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
        pi = np.pi
        theta = state[2]
        deltaX = goal[0] - state[0]
        deltaY = goal[1] - state[1]
        rho = np.sqrt(deltaX**2 + deltaY**2)
        alpha = -theta + np.arctan2(deltaY,deltaX)

        if alpha > pi:
            alpha -= 2*pi
        elif alpha < -pi:
            alpha += 2*pi
        beta = -theta - alpha

        v = self.kp * rho
        omega = self.ka*alpha + self.kb*beta

        # saturate linear velocity
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED
        elif v < self.MIN_SPEED:
            v = self.MIN_SPEED

        # slow down and turn if heading error is too great
        if abs(alpha) > pi/6:
            v = 0

        # saturate angular velocity
        if omega > self.MAX_OMEGA:
            omega = self.MAX_OMEGA
        elif omega < -self.MAX_OMEGA:
            omega = -self.MAX_OMEGA

        # set done to true if robot is close to goal
        if np.abs(rho) < self.tol:
            self.done = True

        return v, omega, self.done
