import numpy as np


def pose_to_tf_matrix(pose: np.ndarray) -> np.ndarray:
    X, Y, THETA = 0, 1, 2
    return np.array([
        [np.cos(pose[THETA]), -np.sin(pose[THETA]), pose[X]],
        [np.sin(pose[THETA]), np.cos(pose[THETA]), pose[Y]],
        [0, 0, 1]
    ])


def tf_matrix_to_pose(mat: np.ndarray) -> np.ndarray:
    return np.array([
        mat[0][2],
        mat[1][2],
        np.arctan2(mat[1][0], mat[0][0])
    ])


def inverse_matrix(matrix: np.ndarray) -> np.ndarray:
    return np.linalg.inv(matrix)
