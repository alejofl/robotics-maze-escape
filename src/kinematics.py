import numpy as np
import typing as Any


def forward_kinematics(vt: float, wt: float, last_pose: np.ndarray, dt: float, dtype: np.dtype = np.float64) -> np.ndarray:
    """
    Forward kinematics for a differential drive robot.
    This function computes the new pose of a differential drive robot given its linear and angular velocities,
    the last known pose, and the time step.

    Args:
        vt (float): The linear velocity of the robot.
        wt (float): The angular velocity of the robot.
        last_pose (np.ndarray): The last known pose of the robot in the form [x, y, theta].
        dt (float): The time step over which the velocities are applied.
        dtype (np.dtype, optional): The data type for the output pose. Defaults to np.float64.

    Returns:
        np.ndarray: The new pose of the robot after applying the velocities for the time step dt, in the form [x, y, theta].
    """
    if not isinstance(last_pose, np.ndarray): 
        last_pose = np.array(last_pose, dtype=dtype)
        assert last_pose.shape == (3,), "Received pose in wrong format! Pose needs to be provided in form [x,y,theta]!"

    if wt == 0: wt = np.finfo(dtype).tiny
    vtwt = vt/wt
    _, _, theta = last_pose
    return last_pose + np.array([
        -vtwt*np.sin(theta) + vtwt*np.sin(theta + (wt*dt)),
        vtwt*np.cos(theta) - vtwt*np.cos(theta + (wt*dt)),
        wt*dt
    ], dtype=dtype)


class PT2Block:
    """
    Discrete PT2 Block approximated using the Tustin approximation.
    """
    def __init__(self, T=0, D=0, kp=1, ts=0, buffer_length=3) -> None:
        self.k1 = 0
        self.k2 = 0
        self.k3 = 0
        self.k4 = 0
        self.k5 = 0
        self.k6 = 0
        self.e = [0 for _ in range(buffer_length)]
        self.y = [0 for _ in range(buffer_length)]
        if ts != 0: self.set_constants(T, D, kp, ts)

    def set_constants(self, T, D, kp, ts):
        self.k1 = 4*T**2 + 4*D*T*ts + ts**2
        self.k2 = 2*ts**2 - 8*T**2
        self.k3 = 4*T**2 - 4*D*T*ts + ts**2
        self.k4 = kp*ts**2
        self.k5 = 2*kp*ts**2
        self.k6 = kp*ts**2

    def update(self, e):
        # Update buffered input and output signals
        self.e = [e]+self.e[:len(self.e)-1]
        self.y = [0]+self.y[:len(self.y)-1]
        # Shorten variable names for better readability
        e = self.e
        y = self.y
        # Calculate output signal and return output
        y[0] = ( e[0]*self.k4 + e[1]*self.k5 + e[2]*self.k6 - y[1]*self.k2 - y[2]*self.k3 )/self.k1
        return y[0]