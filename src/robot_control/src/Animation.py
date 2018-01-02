#!/usr/bin/python
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches

class Animation():
    def __init__(self, markers):

        self.__shapes = [
                [[ 4, 2],[ 4,-2],[-4,-2],[-4, 2]],
                [[-1, 3],[-3, 3],[-3, 2],[-1, 2]],
                [[ 3, 3],[ 1, 3],[ 1, 2],[ 3, 2]],
                [[-1,-3],[-3,-3],[-3,-2],[-1,-2]],
                [[ 3,-3],[ 1,-3],[ 1,-2],[ 3,-2]],
                [[ 4, 2],[ 4,-2],[ 5, 0]],
                ]
        for s in range(len(self.__shapes)):
            for k in range(len(self.__shapes[s])):
                # Resize shapes to match true robot
                self.__shapes[s][k][0] /= 56.25
                self.__shapes[s][k][1] /= 60.

        # How wide to draw the markers in the simulation
        self.__marker_width = 0.02
        # How high to draw the markers in the simulation
        self.__marker_height = 0.075                

        self.markers = markers
        self.markers_flipped = np.copy(self.markers)
        for i in range(self.markers_flipped.shape[0]):
            self.markers_flipped[i][2]+=np.pi
            if self.markers_flipped[i][2] > np.pi:
                self.markers_flipped[i][2]-=2*np.pi
            elif self.markers_flipped[i][2] < np.pi:
                self.markers_flipped[i][2]+=2*np.pi

        self.__visible_markers = [False for i in range(len(self.markers_flipped))]

        self.__view_half_angle = 37*np.pi/180

        self.axes = plt.axes(xlim=(-0.1,1.8),ylim=(-0.1,1.0))
        self.axes.set_aspect('equal')

    def drawMarkers(self):
        self.__markers = [ 0 for i in range(len(self.markers_flipped)) ]
        self.__markers_dir = [ 0 for i in range(len(self.markers_flipped)) ]
        for m in range(len(self.markers_flipped)):
            angle=(self.markers_flipped[m][2]/np.pi)*180.0
            R = np.array([[np.cos(self.markers_flipped[m][2]), -np.sin(self.markers_flipped[m][2])],
                          [np.sin(self.markers_flipped[m][2]), np.cos(self.markers_flipped[m][2])]])
            offset = np.array([[self.__marker_width, self.__marker_height]]).T
            rotated_offset = np.dot(R,offset)
            
            self.__markers[m] = patches.Rectangle(
                (self.markers_flipped[m][0] - rotated_offset[0]/2.,
                 self.markers_flipped[m][1] - rotated_offset[1]/2.),
                self.__marker_width,
                self.__marker_height,
                angle=(self.markers_flipped[m][2]/np.pi)*180.0,
                fill=True,facecolor='r',edgecolor='k')
            
            dir_offset = np.array([[-self.__marker_width, 0.04]]).T
            rotated_dir_offset = np.dot(R, dir_offset)
            self.__markers_dir[m] = patches.Rectangle(
                (self.markers_flipped[m][0] - rotated_dir_offset[0]/2.,
                 self.markers_flipped[m][1] - rotated_dir_offset[1]/2.),
                self.__marker_width*0.5,
                0.04,
                angle=(self.markers_flipped[m][2]/np.pi)*180.0,
                fill=True,facecolor='r',edgecolor='k')
            self.axes.add_patch(self.__markers[m])
            self.axes.add_patch(self.__markers_dir[m])

    def drawRobot(self):
        # Drawing of robot (building drawing objects for it)
        self.__bot_parts = [ 0 for i in range(len(self.__shapes)) ]
        for s in range(len(self.__shapes)):
            self.__bot_parts[s] = patches.Polygon(
                                    self.__shapes[s],
                                    fill=True,facecolor='b',edgecolor='k')
            self.axes.add_patch(self.__bot_parts[s])

        plt.grid(True) # grid on

    def updateRobot(self, state):
        posx,posy,theta = state
        
        # Angles
        cos = math.cos(theta); sin = math.sin(theta)

        # Draw robot (with rectables specified by __shapes)
        for s in range(len(self.__shapes)):
            pts = np.zeros((len(self.__shapes[s]),2))
            for k in range(len(self.__shapes[s])):
                pts[k][0] = posx + cos*self.__shapes[s][k][0] - sin*self.__shapes[s][k][1]
                pts[k][1] = posy + sin*self.__shapes[s][k][0] + cos*self.__shapes[s][k][1]
            self.__bot_parts[s].set_xy(pts)

        plt.pause(0.001) # update plot with robot frame drawn

    def colorVisibleMarkers(self,markers):
        if markers is not None:
            for i in range(len(self.__visible_markers)):
                if i in markers:
                    self.__markers[i].set_facecolor('g')
                    self.__markers_dir[i].set_facecolor('y')
                else:
                    self.__markers[i].set_facecolor('r')
                    self.__markers_dir[i].set_facecolor('r')

    def drawFOV(self,posx,posy,theta):
        # Viewing angle visualization
        distance = 100
        cosx = np.cos(self.__view_half_angle)
        sinx = np.sin(self.__view_half_angle)
        self.__view_lines_1, = plt.plot(
                                [posx, posx + distance*cosx],
                                [posy, posy + distance*sinx],'r-')
        self.__view_lines_2, = plt.plot(
                                [posx, posx + distance*cosx],
                                [posy, posy - distance*sinx],'r-')
    def updateFOV(self, state):
        # Viewing angle visualization
        distance = 2
        posx,posy,theta = state
        cosx = np.cos(self.__view_half_angle + theta)
        sinx = np.sin(self.__view_half_angle + theta)
        cosy = np.cos(-self.__view_half_angle + theta)
        siny = np.sin(-self.__view_half_angle + theta)
        self.__view_lines_1.set_xdata([posx, posx + distance*cosx])
        self.__view_lines_1.set_ydata([posy, posy + distance*sinx])
        self.__view_lines_2.set_xdata([posx, posx + distance*cosy])
        self.__view_lines_2.set_ydata([posy, posy + distance*siny])

    def drawObstacles(self,occupancy_map,x_spacing,y_spacing):
        # Drawing of the map (building drawing objects)
        # Obstacles
        #axes = plt.axes(xlim=(-0.5,2),ylim=(0,1))
        obstacles = [0 for i in range(np.sum(occupancy_map))]
        obs_iter = 0
        for r in range(occupancy_map.shape[0]):
            for c in range(occupancy_map.shape[1]):
                if occupancy_map[r,c]:
                    obstacles[obs_iter] = patches.Rectangle(
                        (c*x_spacing,
                         r*y_spacing),
                        x_spacing,
                        y_spacing,
                        fill=True, facecolor='r', edgecolor='k')
                    self.axes.add_patch(obstacles[obs_iter])
                    obs_iter+=1

    def drawActiveWaypoints(self,pts):
        x, y = zip(*pts)
        
        if hasattr(self, 'h'):
            self.h.remove()

        self.h, = plt.plot(x,y,'ko', markersize=2)
