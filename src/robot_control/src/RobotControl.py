#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""


import yaml
import numpy as np
import sys

if mode == HARDWARE:
    import rospy    
    from RosInterface import ROSInterface

if mode == SIMULATE:
    import RobotSim as RS
    import matplotlib.pyplot as plt

# User files, uncomment as completed
#from MyShortestPath import my_dijkstras
#from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, markers, occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        if mode == HARDWARE:
            # Handles all the ROS related items
            self.ros_interface = ROSInterface(t_cam_to_body)

        if mode == SIMULATE:
            self.robot_sim = RS.RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                         max_speed, max_omega, x_spacing, y_spacing)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        #self.kalman_filter = KalmanFilter(markers)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.markers = markers
        self.goal = pos_goal

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and estimation
        are done. This function is called at 60Hz
        """
        pi = np.pi

        if mode == SIMULATE:
            meas = self.robot_sim.get_measurements() # x,y,theta,id,time
            imu_meas = self.robot_sim.get_imu()
        if mode == HARDWARE:
            meas = self.ros_interface.get_measurements()
            imu_meas = self.ros_interface.get_imu()

        if not meas:
            #print('No tags detected!')
            pass
        else:
            # print(meas)
            meas = np.array(meas) # convert to ndarray for easy slicing
            x,y,theta,id = meas.flat[0:4] # get relative position of first tag
            tagPos = self.markers[id,0:2]
            tagTheta = self.markers[id,2]            

            robotTheta = tagTheta - theta 
            ct,st = np.cos(robotTheta),np.sin(robotTheta)
            Rot = np.array(((ct,-st),(st,ct)))
            pos = np.array((x,y))                        
            robotPos = tagPos - np.dot(Rot,pos)
            
            state = np.append(robotPos,robotTheta)
            if mode == SIMULATE:
                self.robot_sim.set_est_state(state)
            print("X = {} cm, Y = {} cm, Theta = {} deg".format(100*state[0],100*state[1],state[2]*180/pi))

            v,omega,done = self.diff_drive_controller.compute_vel(state, self.goal)
            if not done:
                if mode == HARDWARE:
                    self.ros_interface.command_velocity(v,omega)
                if mode == SIMULATE:
                    self.robot_sim.command_velocity(v,omega)
            else:
                print('We are done!')
                if mode == HARDWARE:                
                    self.ros_interface.command_velocity(0,0)                
                if mode == SIMULATE:
                    self.robot_sim.command_velocity(0,0)
                return
        return
    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)
    
    if mode == HARDWARE:
        # Call process_measurements at 60Hz
        r = rospy.Rate(60)
        while (not rospy.is_shutdown()):
            robotControl.process_measurements()
            r.sleep()
        # Done, stop robot
        robotControl.ros_interface.command_velocity(0,0)

    if mode == SIMULATE:
        # Run the simulation
        while not robotControl.robot_sim.done and plt.get_fignums():
            robotControl.process_measurements()
            robotControl.robot_sim.update_frame()
            wait = input("PRESS ENTER TO CONTINUE.")        

        plt.ioff()
        plt.show()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


