import numpy as np


def pose2tf_mat(pose: np.ndarray):
    X, Y, THETA = 0, 1, 2
    return np.array([
        [np.cos(pose[THETA]), -np.sin(pose[THETA]), pose[X]],
        [np.sin(pose[THETA]), np.cos(pose[THETA]), pose[Y]],
        [0, 0, 1]
    ])

def tf_mat2pose(mat: np.ndarray):
    return np.array([
        mat[0][2],
        mat[1][2],
        np.arctan2(mat[1][0], mat[0][0])
    ])

def inverse_tf_mat(transf_matrix: np.ndarray):
    """Inverts a transformation matrix so that it can be multiplied with another one (inv(robot_position) * next_goal)"""
    return np.linalg.inv(transf_matrix)
