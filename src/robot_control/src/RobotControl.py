#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, markers, occupancy_map, pos_init, pos_goal, max_speed, min_speed, max_omega, x_spacing, y_spacing, t_cam_to_body, mode):
        """
        Initialize the class
        """
        # plan a path around obstacles using dijkstra's algorithm
        print('Planning path...')
        path = findShortestPath(occupancy_map, x_spacing, y_spacing, pos_init[0:2], pos_goal[0:2], dilate = 2)
        print('Done!')
        self.path_manager = PathManager(path)
        self.kalman_filter = KalmanFilter(markers, pos_init)
        self.diff_drive_controller = DiffDriveController(max_speed, min_speed, max_omega)

        if 'HARDWARE' in mode:
            # Handles all the ROS related items
            self.ros_interface = ROSInterface(t_cam_to_body)

        elif 'SIMULATE' in mode:
            self.robot_sim = RobotSim(markers, occupancy_map, pos_init, pos_goal,
                                         max_speed, max_omega, x_spacing, y_spacing, self.path_manager.path, mode)

        #self.user_control = UserControl()
        self.vel = 0 # save velocity to use for kalman filter
        self.goal = self.path_manager.getNextWaypoint() # get the first waypoint

        # for logging postion data to csv file
        self.stateSaved = []
        self.tagsSaved = []
        self.waypoints = []

    def process_measurements(self):
        """ 
        Main loop of the robot - where all measurements, control, and estimation
        are done.
        """
        if 'HARDWARE' in mode:
            meas = self.ros_interface.get_measurements()
            imu_meas = self.ros_interface.get_imu()

        elif 'SIMULATE' in mode:
            meas = self.robot_sim.get_measurements() # x,y,theta,id,time
            imu_meas = self.robot_sim.get_imu()
            

        #meas = None # for testing purposes
        #imu_meas = None # for testing purpose
        #pdb.set_trace()
        if (imu_meas is None) and (meas is None):
            pass
        else:
            state = self.kalman_filter.step_filter(self.vel, imu_meas, meas)
            if 'SIMULATE' in mode:
                self.robot_sim.set_est_state(state)

            #print("X = {} cm, Y = {} cm, Theta = {} deg".format(100*state[0],100*state[1],state[2]*180/np.pi))

            # save the estimated state and tag statuses for offline animation
            self.stateSaved.append(state)
            self.waypoints.append(self.path_manager.getActiveWaypointsPos())
            if meas is None:
                self.tagsSaved.append(None)
            else:
                meas = np.array(meas)
                tagIDs = [int(i) for i in meas[:,3]]
                self.tagsSaved.append(tagIDs)        


            v,omega,wayptReached = self.diff_drive_controller.compute_vel(state, self.goal.pos)
            self.vel=v

            if wayptReached:
                self.goal = self.path_manager.getNextWaypoint()
                self.diff_drive_controller.done = False # reset diff controller status

            if self.goal is None:           
                print('Goal has been reached!')
                np.savez('savedState.npz', stateSaved = self.stateSaved, tagsSaved = self.tagsSaved, waypoints = self.waypoints)
                print('Position data saved!')                
                if 'HARDWARE' in mode:                
                    self.ros_interface.command_velocity(0,0)                
                elif 'SIMULATE' in mode:

                    self.robot_sim.command_velocity(0,0)
                    self.robot_sim.done = True
                return                    
            else:
                if 'HARDWARE' in mode:
                    self.ros_interface.command_velocity(self.vel,omega)
                elif 'SIMULATE' in mode:
                    self.robot_sim.command_velocity(self.vel,omega)
        return

    def myhook():
        print "shutdown time!"
    
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
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    world_map[:,0:2] += 0.08 # offset because I want origin to be located at bottom-left corner of table, not on tape
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
        r = rospy.Rate(10)
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

    # if len(sys.argv) > 1:
    #     if sys.argv[1] == "SIMULATE":
    #         mode = "SIMULATE"
    #     elif sys.argv[1] == "HARDWARE":
    #         mode = "HARDWARE"
    # else:
    #     mode = "HARDWARE" # default to mode=HARDWARE
    

    print('Importing libraries...')
    import KalmanFilter as KF
    import DiffDriveController as DFC
    import UserControl as UC

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
    print('Done!')    

    if 'HARDWARE' in mode:
        try:
            main(sys.argv)
        except rospy.ROSInterruptException: pass

    elif 'SIMULATE' in mode:
        main(sys.argv)
