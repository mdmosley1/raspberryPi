import csv
import time
import yaml
import numpy as np
from Animation import Animation

file = '../params/params.yaml'
f = open(file,'r')
params_raw = f.read()
f.close()

params = yaml.load(params_raw)
markers = np.array(params['world_map'])
markers[:,0:2] += 0.08 # offset so that markers are not inside the obstacles
occupancy_map = np.array(params['occupancy_map'])
x_spacing = params['x_spacing']
y_spacing = params['y_spacing']
pos_init = np.array(params['pos_init'])
posx,posy,theta = pos_init

f = np.load('savedState.npz')
stateSaved = f['stateSaved']
tagsSaved = f['tagsSaved']
waypoints = f['waypoints']

animate = Animation(markers)
animate.drawRobot()
animate.drawMarkers()
animate.drawObstacles(occupancy_map,x_spacing,y_spacing)
animate.drawFOV(posx,posy,theta)

N = stateSaved.shape[0]

for n in range(N):
    state = stateSaved[n,:]
    tags = tagsSaved[n]
    activePts = waypoints[n]
    
    animate.updateRobot(state)
    animate.updateFOV(state)
    animate.colorVisibleMarkers(tags)
    animate.drawActiveWaypoints(activePts)

    #print(waypoints[n])
    #time.sleep(0.01)
