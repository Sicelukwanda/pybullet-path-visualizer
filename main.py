import pybullet as p
import pybullet_data
import math
import time
import os
import numpy as np
from mesh_loaders import Mesh  
from robots.utils import generate_orientation_vectors
import matplotlib.cm as cm

# import specific robot kinematics utils
from robots.panda import PandaForwardKinematics

# Instantiate forward kinematics object
fk = PandaForwardKinematics()

# load & process data
best = np.load("data/best_sample.npy")
sigma = np.load("data/uncertainty.npy")
samples = np.load("data/samples.npy")
mu = np.mean(samples, axis=0)
eef_paths = np.array(list(map(fk, samples)))
samples_std = np.std(eef_paths,axis=0)
samples_mu = np.mean(eef_paths,axis=0)
xyz_mu = fk(mu)
xyz_best = fk(best)


# Set up the simulation
physicsClient = p.connect(
    p.GUI,
    options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0'
    )

# disable GUI (axes indicator + side panels)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# set physics parameters
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)

plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

# add additional pybullet search paths
dir_path = os.path.dirname(os.path.realpath(__file__))
p.setAdditionalSearchPath(dir_path)

num_waypoints = int(mu.shape[0])
T = np.linspace(0.0, 1.5*np.pi, num_waypoints)
# sizes = np.linspace(0.5, 2.0, num_waypoints)
# create a list of meshes
meshes = []
SCALE = 2.0 # 2x standard deviation

path_ori = generate_orientation_vectors(samples_mu)
# define cmap inputs ()
total_uncertainty = np.sum(samples_std,axis=1)
total_uncertainty = total_uncertainty / np.max(total_uncertainty)


for i in range(0,num_waypoints-1):
    print("Adding mesh {}".format(i))
    pos = samples_mu[i]
    color = cm.viridis(total_uncertainty[i])[:3]
    print(color)
    mesh = Mesh(
        scale = samples_std[i]*SCALE, 
        positionXYZ = pos, 
        orientationXYZ = path_ori[i]*np.array([1.0,1.0,1.0]), # also rotate the mesh by t about the z-axis
        rgbColor =color,
        alpha = 0.4
        )
    meshes.append(mesh)
    # time.sleep(0.2)

# step through the simulation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# 