
import numpy as np
from robots.utils import getBaseTransformMatrix, getBaseTransformMatrixBatch, getModifiedTransformMatrix

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
