#!/usr/bin/env python
"""ROS based interface for the Course Robotics Specialization Capstone
Autonomous Rover.  Updated June 15 2016.

"""
class RobotControl(object):

    """Class used to interface with the rover. Gets sensor measurements
    through ROS subscribers, and transforms them into the 2D plane,
    and publishes velocity commands.

    """
    def __init__(self, markers, occupancy_map, pos_init, pos_goal,
                 max_speed, min_speed, max_omega, x_spacing, y_spacing,
                 t_cam_to_body, mode):
        """
        Initialize the class
        """
        # plan a path around obstacles using dijkstra's algorithm
        print('Planning path...')
        path = findShortestPath(occupancy_map, x_spacing, y_spacing,
                                pos_init[0:2], pos_goal[0:2], dilate = 2)
        print('Done!')
        self.path_manager = PathManager(path)
        self.kalman_filter = KalmanFilter(markers, pos_init)
        self.diff_drive_controller = DiffDriveController(max_speed,
                                                         min_speed, max_omega)

        if 'HARDWARE' in mode:
            # Handles all the ROS related items
            self.ros_interface = ROSInterface(t_cam_to_body)

        elif 'SIMULATE' in mode:
            self.robot_sim = RobotSim(markers, occupancy_map,
                                      pos_init, pos_goal, max_speed,
                                      max_omega, x_spacing, y_spacing,
                                      self.path_manager.path, mode)

        self.user_control = UserControl()
        self.vel = 0  # save velocity to use for kalman filter
        self.goal = self.path_manager.getNextWaypoint()  # get first waypoint

        # for logging postion data to csv file
        self.stateSaved = []
        self.tagsSaved = []
        self.waypoints = []

    def process_measurements(self):
        """Main loop of the robot - where all measurements, control, and
        estimation are done.

        """
        v, omega, wayptReached = self.user_control.compute_vel()
        self.robot_sim.command_velocity(v, omega)
        return

def main(args):
    if 'HARDWARE' in mode:
        rospy.init_node('robot_control')
        param_path = rospy.get_param("~param_path")

    if 'SIMULATE' in mode:
        param_path = '../params/params.yaml'

    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    #occupancy_map = np.array(params['occupancy_map'])
    f2 = np.load('savedMaps.npz')
    occupancy_map = f2['kitchenMap']
    world_map = np.array(params['world_map'])
    pos_init = params['pos_init']
    pos_goal = params['pos_goal']
    max_vel = params['max_vel']
    min_vel = params['min_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal,
                                max_vel, min_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body, mode)

    if 'HARDWARE' in mode:
        # Call process_measurements at 60Hz
        r = rospy.Rate(60)
        while (not rospy.is_shutdown()):
            robotControl.process_measurements()
            r.sleep()
        # Done, stop robot
        robotControl.ros_interface.command_velocity(0,0)

    elif 'SIMULATE' in mode:
        # Run the simulation
        while not robotControl.robot_sim.done and plt.get_fignums():
            robotControl.process_measurements()
            robotControl.robot_sim.update_frame()
            #wait = input("PRESS ENTER TO CONTINUE.")

        plt.ioff()
        plt.show()

if __name__ == "__main__":

    import sys
    mode = sys.argv # keep track of what options I pass to RobotControl

    """
    RobotControl.py can take a few optional arguments
         SIMULATE: instruct program to use RobotSim class to simulate robot
         HARDWARE: instruct program to use RosInterface class to use hardware
    """


    print('Importing libraries...')
    import KalmanFilter as KF
    import DiffDriveController as DFC
    import yaml
    import numpy as np


    if 'HARDWARE' in mode:
        import rospy
        from RosInterface import ROSInterface

    elif 'SIMULATE' in mode:
        from RobotSim import RobotSim
        import matplotlib.pyplot as plt

    from PathPlanner.ShortestPath import findShortestPath
    from KalmanFilter import KalmanFilter
    from DiffDriveController import DiffDriveController
    from PathManager import PathManager
    from UserControl import UserControl

    print('Done!')

    if 'HARDWARE' in mode:
        try:
            main(sys.argv)
        except rospy.ROSInterruptException: pass

    elif 'SIMULATE' in mode:
        main(sys.argv)
