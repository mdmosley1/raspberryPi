#!/usr/bin/python
import matplotlib.pyplot as plt

class PathManager:
    """
    Class used for managing waypoints on the robot path
    wypts: list of tuples representing x,y positions of each waypoint
    """
    def __init__(self, wypts):
        # keep list of Waypoint objects to represent the path
        self.path = []
        self.waypointIdx = -1
        for pos in wypts:
            newWaypoint = Waypoint(pos)
            self.path.append(newWaypoint)

    def getNextWaypoint(self):
        if self.waypointIdx > -1:
            self.path[self.waypointIdx].status = 'Marked'

        self.waypointIdx += 1
        if self.waypointIdx < len(self.path):
            nextWaypoint = self.path[self.waypointIdx]
        else:
            nextWaypoint = None

        return nextWaypoint

    def getActiveWaypointsPos(self):
        activeWaypoints = []
        for wp in self.path:
            if wp.status == 'Active':
                activeWaypoints.append(wp.pos)

        return activeWaypoints

class Waypoint:

    def __init__(self,pos):
        self.status = 'Active' # keep track of whether waypoing it active for visual purposes
        self.pos = pos

    def plot(self,ax):
        self.h, = ax.plot(self.pos[0], self.pos[1], 'ko', markersize=2)

    def markAsInactive(self):
        #plt.plot(self.pos[0], self.pos[1], 'wo', markersize=2)
        self.h.remove()
        self.status = 'Inactive'

