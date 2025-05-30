
import copy
from custom_types import Point
from emission import RobotMovementEmitter
from kinematics import PT2Block, forwardKinematics
from matrix_manipulation import inverse_tf_mat, pose2tf_mat, tf_mat2pose
import rospy
import tf2_ros
from nav_msgs.msg import Path as RosPath
import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import PoseStamped as RosPoseStamped

from scipy.spatial.transform import Rotation as R


class RobotMovement:

    def __init__(self, initial_position: Point, initial_orientation, goals):
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_pose = [initial_position.x, initial_position.y, initial_orientation]
        
        # print("Waiting for global plan...")
        # def callback(data):
        #     print("callback!!!")
        #     self.got_info = True
        #     self.goals = [(pose.position.x, pose.position.y, 0.0) for pose in data.poses]
        #     print(f"Received {len(self.goals)} goals")
        # rospy.sleep(1)
        # rospy.Subscriber("/maze_escape/global_plan", RosPath, callback)
        
        self.goals = []
        # TODO: LOS GOALS AHORA ESTÁN COMO MapPoint, NO COMO PoseStamped. NOS FALTAN LOS THETA
        #for goal in goals:
            # curr_goal = RosPoseStamped()
            # curr_goal.pose.position.x = goal.x
            # curr_goal.pose.position.y = goal.y
            # curr_goal.pose.position.z = 0.0
        #self.goals.append(np.array([goal.x, goal.y, 0.0]) for goal in goals)
        self.goals.append(np.array([3,1,0]))
        self.goals.append(np.array([3,2,0]))
        self.goals.append(np.array([2,2,0]))
        
        print(f"Received {goals} goals") # BORRAR ESTO
        print(f"Goals turned into poses: {self.goals}")
        
        self.current_goal_index = 0
        self.movpub = RobotMovementEmitter()
        print(f"Robot initial position: {self.robot_pose}")

    def _localiseRobot(self):
        """Localises the robot towards the 'map' coordinate frame. Returns pose in format (x,y,theta)"""
        while True:
            try:
                trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(f"Robot localisation took longer than 1 sec, {e}")
                continue

        theta = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w]).as_euler("xyz")[2]
        
        self.robot_pose = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            theta])

    def _get_goal_in_robot_coordinates(self):
        """Returns the goal in the robot coordinates"""
        robot_tf_mat = pose2tf_mat(self.robot_pose)
        goal_tf_mat = pose2tf_mat(self.goals[self.current_goal_index])  # Esto en realidad es de array a tf_mat, pero se entiende
        goal_in_robot_coordinates = inverse_tf_mat(robot_tf_mat) @ goal_tf_mat # Por qué era un arroba para multiplicar matrices no?
        return tf_mat2pose(goal_in_robot_coordinates)

    def _create_vt_and_wt(self, previous_vt, previous_wt):
        values = []
        for vt in np.arange(-0.2, 0.2 +0.1, 0.1):
            for wt in np.arange(-10, 10 +0.1, 0.5):
                values.append((vt, wt))
        return values

    def _costFn(self, pose: npt.ArrayLike, goalpose: npt.ArrayLike, control: npt.ArrayLike) -> float:
        x_error = np.abs(pose[0] - goalpose[0])
        y_error = np.abs(pose[1] - goalpose[1])
        theta_error = np.abs(pose[2] - goalpose[2])
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
            [0, 0, 0.0] # Podría ser 0.3
        ])

        c_weight_matrix = np.array([
            [0, 0],
            [0, 0]  # Sign that the horizon should be higher
        ])

        return np.transpose(error_vector) @ s_weight_matrix @ error_vector + np.transpose(control) @ c_weight_matrix @ control

    # We directly check the goalpose in the robot coordinates
    def _is_goal_reached(self, goalpose: npt.ArrayLike) -> bool:
        #robot_pose = self.robot_pose
        #x_error = np.abs(robot_pose[0] - goalpose[0])
        #y_error = np.abs(robot_pose[1] - goalpose[1])
        #theta_error = np.abs(robot_pose[2] - goalpose[2])
        #if theta_error > np.pi:
        #    theta_error -= 2*np.pi
        #elif theta_error < -np.pi:
        #    theta_error += 2*np.pi
        #print(f"Robot pose: {robot_pose}, Goal pose: {goalpose}, Errors: x={x_error}, y={y_error}, theta={theta_error}")
        #return x_error < 0.1 and y_error < 0.1 and theta_error < 0.1
        #print(f"Goal pose: {goalpose}")
        return goalpose[0] < 0.1 and goalpose[1] < 0.1# and np.abs(goalpose[2]) < 0.2

    def run_robot(self, robotModelPT2: PT2Block, horizon, ts: float):
        first = True
        while True:
            curr_goal_pose = self._get_goal_in_robot_coordinates()
            print(f"Current goal pose: {curr_goal_pose}")
            print_counter = 0
            while not self._is_goal_reached(curr_goal_pose):
                if not first:
                    self._localiseRobot()
                    curr_goal_pose = self._get_goal_in_robot_coordinates()
                    #print(f"Robot pose: {self.robot_pose}")
                else:
                    print(f"Robot pose: {self.robot_pose}")
                    curr_vt = 0
                    curr_wt = 0
                    first = False
                
                costs = []
                index = 0
                min_cost_index = 0
                min_cost = float("inf")
                print_counter += 1
                for control in self._create_vt_and_wt(curr_vt, curr_wt):
                    forwardSimPT2 = copy.deepcopy(robotModelPT2)
                    forwardpose = [0,0,0]
                    curr_cost = 0
                    for i in range(horizon):
                        control_sim = copy.deepcopy(control)
                        v_t, w_t = control
                        vt_dynamic = forwardSimPT2.update(v_t)
                        control_dym = [vt_dynamic, w_t]
                        forwardpose = forwardKinematics(control_dym, forwardpose, ts)
                        curr_cost += self._costFn(forwardpose, curr_goal_pose, control_sim)
                    if curr_cost < min_cost:
                        min_cost = curr_cost
                        min_cost_index = index
                    costs.append((control_sim, curr_cost))   # This data is useful for plotting
                    index += 1
                curr_vt, curr_wt = costs[min_cost_index][0][0], costs[min_cost_index][0][1]
                #if print_counter%10 == 0:
                #    print(f"Current vt: {curr_vt}, Current wt: {curr_wt}, Cost: {min_cost}")
                    #print(f"Similar costs: {costs[min_cost_index-2:min_cost_index+3]}")
                    #print(f"costs: {costs}")
                #    print(f"Current goal pose: {curr_goal_pose}")
                
                self.movpub.emit([curr_vt, curr_wt])

            print(f"Goal of index {self.current_goal_index} reached")
            self.current_goal_index += 1
            if self.current_goal_index >= len(self.goals):
                self.movpub.emit([0, 0])
                print(f"Goal of index {self.current_goal_index-1} reached")
                break

