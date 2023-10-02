import pybullet as p
import pybullet_data
import math
import time
import os
import numpy as np
from mesh_loaders import Mesh  

import matplotlib.cm as cm
dir_path = os.path.dirname(os.path.realpath(__file__))

def generate_orientation_vectors(eef_path, window_size=10):
    """
    Given a path of end effector positions, generate a path of orientation vectors
    """
    num_waypoints = eef_path.shape[0]
    path_ori = np.zeros((num_waypoints,3))
    for i in range(0,num_waypoints-1):
        if i < window_size:
            path_ori[i] = eef_path[i+1] - eef_path[i]
        else:
            path_ori[i] = eef_path[i+1] - eef_path[i-window_size]
    path_ori[-1] = path_ori[-2]
    # normalize
    path_ori = path_ori / np.linalg.norm(path_ori, axis=1)[:,np.newaxis]
    return path_ori


def getBaseTransformMatrix(t):
    r""" get base for DH transforms. Invariant to rotation.

    Args:
        t (np.array): xyz coordinate of the base for DH chain transforms

    Returns:
        [np.array]: (1,4,4) translation matrix
    """

    T = np.array([
        [1., 0., 0., t[0]],
        [0., 1., 0., t[1]],
        [0., 0., 1., t[2]],
        [0., 0., 0., 1.]
    ],dtype=np.float64)

    return T[np.newaxis,:]

def getBaseTransformMatrixBatch(t, batch_zeros, batch_ones):
    r""" get base for DH transforms. Invariant to rotation.

    Args:
        t (np.array): xyz coordinate of the base for DH chain transforms

    Returns:
        [np.array]: (B,4,4) translation matrix
    """

    T = np.stack([
        np.stack([batch_ones, batch_zeros, batch_zeros, t[:,0]], axis=1),
        np.stack([batch_zeros, batch_ones, batch_zeros, t[:,1]], axis=1),
        np.stack([batch_zeros, batch_zeros, batch_ones, t[:,2]], axis=1),
        np.stack([batch_zeros, batch_zeros, batch_zeros, batch_ones], axis=1)
    ],axis=1)

    return T
    
def getModifiedTransformMatrix(batch_thetas, a, d, alpha, batch_zeros, batch_ones):
    r"""
    Returns 4x4 homogenous matrix from Modified DH parameters for batch of thetas (i.e., single joint angle).
    """
    cTheta = np.cos(batch_thetas) 
    sTheta = np.sin(batch_thetas)

    # TODO: place these outside
    calpha = np.cos(alpha*batch_ones)
    salpha = np.sin(alpha*batch_ones)


    T = np.stack([
        np.stack([cTheta, -sTheta, batch_zeros, a*batch_ones], axis=1),
        np.stack([ calpha * sTheta, calpha * cTheta, -salpha, -d * salpha], axis=1),
        np.stack([salpha * sTheta, salpha * cTheta, calpha, d * calpha], axis=1),
        np.stack([batch_zeros, batch_zeros, batch_zeros, batch_ones], axis=1)
    ], axis=1)

    return T
class PandaForwardKinematics():
    def __init__(self):
        self.BasePos = np.array([0.0, 0.0, 0.0])
        self.BaseRot = np.array([0.0, 0.0, 0.0, 1.0])
        self.DH = np.array([
            [0,  0.333,  0,           ],   #r_shoulder_pan_joint
            [0,  0,     -1.5708,      ],   # r_shoulder_lift_joint
            [0,  0.316,     1.5708,   ],      # r_upper_arm_joint
            [0.0825,  0,      1.5708, ],        # r_elbow_flex_joint
            [-0.0825,  0.384, -1.5708,],         # r_forearm_roll_joint
            [0,  0,      1.5708,      ],   # r_wrist_flex_joint
            [0.088,  0,        1.5708,],          # r_wrist_roll_joint
            [0,  0.107,        0      ]  # flange
        ], dtype=np.float64)

    def __call__(self, joint_states, base_positions=None):
        """
        Calculates forward kinematics for batch of joint states (shape: B x D)
        B - batch size
        D - Dimensionality or DOF

        returns batch of XYZ positions for the end effector.
        """
        if len(joint_states.shape) == 1:
            joint_states = joint_states[np.newaxis,:]
        
        B, D = joint_states.shape

        batch_ones = np.ones(B)
        batch_zeros = np.zeros(B)
        
        if base_positions is None:
            base_transform_mat = getBaseTransformMatrix(self.BasePos)
            H_transform = np.repeat(base_transform_mat, B, axis=0)
        else:
            H_transform = getBaseTransformMatrixBatch(base_positions, batch_zeros, batch_ones)

        for i in range(7):
            Theta_Transform = getModifiedTransformMatrix(joint_states[:,i], self.DH[i,0], self.DH[i,1], self.DH[i,2], batch_zeros, batch_ones)
            H_transform = np.matmul(H_transform, Theta_Transform)

        # last transformation for flange (thetas = batch zeros)
        Theta_Transform = getModifiedTransformMatrix(batch_zeros, self.DH[-1,0], self.DH[-1,1], self.DH[-1,2], batch_zeros, batch_ones)
        H_transform = np.matmul(H_transform, Theta_Transform)
        
        return H_transform[:,0:3,-1]

# Instantiate forward kinematics object
fk = PandaForwardKinematics()

# load data
best = np.load("data/best_sample.npy")
mu = np.load("data/mu.npy")
sigma = np.load("data/uncertainty.npy")
samples = np.load("data/samples.npy")
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

# set physics parameters
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)

plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

# add additional pybullet search paths
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