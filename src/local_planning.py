import copy
from custom_types import Pose
from emission import RobotMovementEmitter, TrajectoryEmitter
from kinematics import PT2Block, forward_kinematics
from matrix_manipulation import inverse_matrix, pose_to_tf_matrix, tf_matrix_to_pose
import rospy
import tf2_ros
import numpy as np
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R


class RobotMovement:
    """
    Class for local planning of a robot's movement towards a series of goals.
    This class uses a PT2 block for velocity control and computes the robot's trajectory
    based on the current pose and a set of goals.
    """
    def __init__(
        self,
        initial_pose: Pose,
        goals: np.ndarray,
        time_step: float,
        horizon: int,
        velocity_publisher: RobotMovementEmitter,
        trajectory_publisher: TrajectoryEmitter
    ):
        """
        Initializes the RobotMovement class.

        Args:
            initial_pose (Pose): Initial pose of the robot in the form of a Pose object.
            goals (np.ndarray): Array of goals where each goal is a Point with x, y coordinates.
            time_step (float): Time step for the PT2 block and trajectory calculations.
            horizon (int): Number of steps in the future to consider for trajectory planning.
            velocity_publisher (RobotMovementEmitter): Publisher for robot velocity messages.
            trajectory_publisher (TrajectoryEmitter): Publisher for local trajectory messages.
        """
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot_pose = (initial_pose.x, initial_pose.y, initial_pose.theta)
        
        self.goals = []
        for i, goal in enumerate(goals):
            if i == len(goals) - 1:
                self.goals.append(np.array([goal.x, goal.y, 0.0]))
            else:
                theta = np.arctan2(goals[i + 1].y - goal.y, goals[i + 1].x - goal.x)
                self.goals.append(np.array([goal.x, goal.y, theta]))
        self.current_goal_index = 0

        self.time_step = time_step
        self.pt2_block = PT2Block(ts=time_step)
        self.horizon = horizon

        self.velocity_publisher = velocity_publisher
        self.trajectory_publisher = trajectory_publisher

    def localize_robot(self):
        """
        Private method.
        Localizes the robot by looking up its transform from the 'map' frame to the 'base_link' frame.
        This method continuously attempts to retrieve the robot's pose until successful.
        It updates the robot_pose attribute with the current position and orientation of the robot.
        
        Returns:
            None
        """
        while True:
            try:
                trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue

        theta = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ]).as_euler("xyz")[2]

        self.robot_pose = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            theta
        ])

    def get_goal_in_robot_coordinates(self) -> np.ndarray:
        """
        Private method.
        Converts the current goal pose from world coordinates to robot coordinates.
        This method computes the transformation matrix from the robot's pose to the goal's pose
        and returns the goal pose in the robot's coordinate system.

        Returns:
            np.ndarray: The goal pose in the robot's coordinate system as a numpy array.
        """
        robot_tf_matrix = pose_to_tf_matrix(self.robot_pose)
        goal_tf_matrix = pose_to_tf_matrix(self.goals[self.current_goal_index])
        goal_in_robot_coordinates = inverse_matrix(robot_tf_matrix) @ goal_tf_matrix # The 'at' operator is used for matrix multiplication
        return tf_matrix_to_pose(goal_in_robot_coordinates)
    
    def get_pose_in_world_coordinates(self, pose: np.ndarray) -> np.ndarray:
        """
        Private method.
        Converts a given pose from robot coordinates to world coordinates.
        This method computes the transformation matrix from the robot's pose to the given pose
        and returns the pose in world coordinates.

        Args:
            pose (np.ndarray): The pose in robot coordinates as a numpy array.

        Returns:
            np.ndarray: The pose in world coordinates as a numpy array.
        """
        robot_tf_matrix = pose_to_tf_matrix(self.robot_pose)
        pose_tf_matrix = pose_to_tf_matrix(pose)
        pose_in_world_coordinates = robot_tf_matrix @ pose_tf_matrix
        return tf_matrix_to_pose(pose_in_world_coordinates)

    def create_vt_and_wt(self) -> List[Tuple[float, float]]:
        """
        Private method.
        Creates a list of tuples representing possible forward velocities (vt) and angular velocities (wt).

        Returns:
            List[Tuple[float, float]]: A list of tuples where each tuple contains a linear velocity and an angular velocity.
        """
        values = []
        for vt in np.arange(-0.2, 0.2 +0.1, 0.1):
            for wt in np.arange(-10, 10 +0.1, 0.5):
                values.append((vt, wt))
        return values

    def cost_function(self, pose: np.ndarray, goal_pose: np.ndarray, control: np.ndarray) -> float:
        """
        Private method.
        Computes the cost of a given pose relative to the goal pose and control input.
        This function calculates the error in position and orientation between the current pose and the goal pose,
        and applies a weighted sum to compute the cost. The velocity is also considered in the cost calculation.

        Args:
            pose (np.ndarray): Position and orientation of the robot in the form of a numpy array.
            goal_pose (np.ndarray): Goal position and orientation in the form of a numpy array.
            control (np.ndarray): Velocity in the form of a numpy array, containing linear and angular components.

        Returns:
            float: The computed cost as a float value.
        """
        x_error = np.abs(pose[0] - goal_pose[0])
        y_error = np.abs(pose[1] - goal_pose[1])
        theta_error = np.abs(pose[2] - goal_pose[2])
        if theta_error > np.pi:
            theta_error -= 2*np.pi
        elif theta_error < -np.pi:
            theta_error += 2*np.pi
        
        error_vector = np.array([
            x_error,
            y_error,
            theta_error
        ])

        s_weight_matrix = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.0] # Could be 0.3
        ])

        c_weight_matrix = np.array([
            [0, 0],
            [0, 0]
        ])

        return np.transpose(error_vector) @ s_weight_matrix @ error_vector + np.transpose(control) @ c_weight_matrix @ control

    def is_goal_reached(self, goal_pose: np.ndarray) -> bool:
        """
        Private method.
        Checks if the robot has reached the current goal pose.
        
        Returns:
            bool: True if the robot is within a threshold distance from the goal pose, False otherwise.
        """
        return goal_pose[0] < 0.1 and goal_pose[1] < 0.1 #and np.abs(goal_pose[2]) < 0.2

    def run(self):
        """
        The main method that runs the local planning algorithm.
        It continuously checks if the robot has reached the current goal pose and updates the robot's trajectory
        towards the goal using a PT2 block for velocity control.
        The method iterates through the goals, localizing the robot and computing the optimal control inputs
        to minimize the cost function until all goals are reached.
        The trajectory is published at each step, and the robot's velocity is updated accordingly.
        
        Returns:
            None
        """
        while True:
            current_goal_pose = self.get_goal_in_robot_coordinates()
            while not self.is_goal_reached(current_goal_pose):
                self.localize_robot()
                current_goal_pose = self.get_goal_in_robot_coordinates()

                min_cost = np.inf
                min_forward_poses = None
                min_control = None

                for control in self.create_vt_and_wt():
                    forward_velocity = copy.deepcopy(self.pt2_block)
                    forward_pose = [0 ,0, 0]
                    current_cost = 0
                    forward_poses = []
                    for i in range(self.horizon):
                        vt, wt = control
                        v_predicted = (forward_velocity.update(vt), wt)
                        forward_pose = forward_kinematics(*v_predicted, forward_pose, self.time_step)
                        forward_poses.append(forward_pose)
                        current_cost += self.cost_function(forward_pose, current_goal_pose, control)
                    if current_cost < min_cost:
                        min_cost = current_cost
                        min_control = control
                        min_forward_poses = forward_poses

                self.trajectory_publisher.emit([Pose(*self.get_pose_in_world_coordinates(p)) for p in min_forward_poses])
                self.velocity_publisher.emit(min_control)

            self.current_goal_index += 1
            if self.current_goal_index >= len(self.goals):
                self.velocity_publisher.emit([0, 0])
                break
