import pybullet as p
import pybullet_data
import math
import time
import os
import numpy as np
from mesh_loaders import Mesh  

dir_path = os.path.dirname(os.path.realpath(__file__))

# generate example path
def circle3D(t, start_pos, returnVel=True):
    """
    Given parameter t, returns a 3D position (x,y,z) and  that lies on a circle in cartesian space 
    """

    p = np.array(start_pos) # starting position
    r = 1.0 # radius of circle

    v1 = np.array([1,0,0])
    v2 = np.array([0,1,0])
    
    pos = p - np.array([r,0,0]) + r*np.cos(t)*v1 + r*np.sin(t)*v2
    return pos

# Set up the simulation
physicsClient = p.connect(
    p.GUI,
    options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0'
    )

# set physics parameters
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)

plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

# add additional pybullet search paths
p.setAdditionalSearchPath(dir_path)

num_waypoints = 60
T = np.linspace(0.0, 1.5*np.pi, num_waypoints)
sizes = np.linspace(0.5, 2.0, num_waypoints)
# create a list of meshes
meshes = []
for i in range(num_waypoints):
    pos = circle3D(T[i], [0,0,1])
    mesh = Mesh(
        scale = sizes[i]*np.array([0.1, 0.1, 0.1]), 
        positionXYZ = pos, 
        orientationXYZ = [0, 0, T[i]], # also rotate the mesh by t about the z-axis
        alpha = 1.0
        )
    meshes.append(mesh)

# step through the simulation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# 