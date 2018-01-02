#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math
import pdb

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers, initial_state):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = 25*np.eye(2)   # process covariance
        self.R_t = np.eye(3)   # measurement covariance
        self.P_t = np.eye(3)   # error covariance
        self.x = initial_state # the estimated state

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        xp - a 3 by 1 numpy array of the prediction of the state
        Pp - a 3 by 3 numpy array of the prediction of the covariance
        """
        omega = imu_meas[3]
        current_time = imu_meas[4]
        theta = self.x[2]

        if self.last_time == None:
            dt = 0
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        
        dfdx = np.eye(3) + dt*np.array([(0, 0,-v*np.sin(theta)), \
                                        (0, 0, v*np.cos(theta)), \
                                        (0, 0, 0)]) # jacobian of state transition function
        dfdn = dt*np.array(( (math.cos(theta), 1), (math.sin(theta), 1), (1, 1) ))
        xp = self.x + dt*np.array((v*np.cos(theta), v*np.sin(theta), omega))
        Pp = np.dot(np.dot(dfdx,self.P_t), dfdx.T) + np.dot(np.dot(dfdn, self.Q_t), dfdn.T)
        
        return xp,Pp


    def update(self,z_t,xp,Pp):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        meas - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        x - a 3 by 1 numpy array of the updated state
        """
        tmp = np.linalg.inv(Pp + self.R_t)
        K = np.dot(Pp, tmp)                  # II compute kalman gain
        x = xp + np.dot(K,z_t - xp)          # III compute state estimate
        P = Pp - np.dot(K,Pp)                # IV compute error covariance

        return x,P

    def transformMeasurement(self, meas):
        meas = np.array(meas) # convert to ndarray for easy slicing
        numberOfTags = meas.shape[0]

        if numberOfTags == 0:
            raise ValueError('Number of Tags should never be zero!')

        # make z_t = weighted average of april tag positions, weighted based on which tag is most directly facing
        # the camera            
        z_t = np.array((0,0,0)) # initialize measured state
        weightsSum = 0          # sum of weights for taking weighted average
        for tagNum in range(0,numberOfTags):
            x,y,theta,tagID = meas[tagNum, 0:4] # unpack values for this tag
            tagID = int(tagID)

            tagPos = self.markers[tagID,0:2]
            tagTheta = self.markers[tagID,2]

            robotTheta = tagTheta - theta 
            ct,st = np.cos(robotTheta),np.sin(robotTheta)
            Rot = np.array(((ct,-st),(st,ct)))
            pos = np.array((x,y))                        
            robotPos = tagPos - np.dot(Rot,pos)

            weight = 1 / abs(theta) # weight this measurement more heavily if theta is small
            weightsSum += weight
            z_t = z_t + np.append(robotPos,robotTheta) * weight #the robot state according to apriltag measurement
            
        z_t = z_t / weightsSum # normalize average
        z_t = z_t.tolist()
        return z_t
        
    def step_filter(self, v, imu_meas, meas):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x - current estimate of the state
        """

        if (imu_meas is not None) and (meas is None):
            xp,Pp = self.prediction(v,imu_meas)
            self.x = xp; self.P_t = Pp
            return xp

        elif (imu_meas is None) and (meas is not None):
            z_t = self.transformMeasurement(meas)
            self.x = z_t
            return z_t

        else: # both measurements contain values
            xp,Pp = self.prediction(v,imu_meas)            
            z_t = self.transformMeasurement(meas)
            x,P = self.update(z_t, xp, Pp)

            self.x = x; self.P_t = P
            return x
