
import numpy as np

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
