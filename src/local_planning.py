
import rospy
import tf2_ros

from scipy.spatial.transform import Rotation as R

rospy.init_node("local_planner") # <- If not already initialised, create your node here
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)


class RobotMovement:

    

    # Podemos hacer que el robot reciba los goals, en lugar de emitirlos en un topic
    def __init__(self, initial_position: Point, initial_orientation: float, goals: np.ndarray):
        self.position = initial_position
        self.orientation = initial_orientation
        self.vt = 0.0
        self.wt = 0.0
        self.goals = goals
        self.current_goal_index = 0

    def _localiseRobot():
        """Localises the robot towards the 'map' coordinate frame. Returns pose in format (x,y,theta)"""
        while True:
            try:
                trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(f"Robot localisation took longer than 1 sec, {e}")
                continue

        theta = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w]).as_euler("xyz")[2]
        
        return np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            theta])


    def pose2tf_mat(pose: np.Array):
        X = 0, Y = 1, THETA = 2
        return np.array([
            [np.cos(pose[THETA]), -np.sin(pose[THETA]), pose[X]],
            [np.sin(pose[THETA]), np.cos(pose[THETA]), pose[Y]],
            [0, 0, 1]
        ])

    def tf_mat2pose(mat: np.Array):
        return np.array([
            mat[0][2],
            mat[1][2],
            np.arctan2(mat[1][0], mat[0][0])
        ])

    def inverse_tf_mat(transf_matrix: np.Array):
        """Inverts a transformation matrix so that it can be multiplied with another one (inv(robot_position) * next_goal)"""
        return np.linalg.inv(transf_matrix)

    def get_goal_in_robot_coordinates(self):
        """Returns the goal in the robot coordinates"""
        robot_pose = self._localiseRobot() # No estariamos usando el inicial
        robot_tf_mat = self.pose2tf_mat(robot_pose)
        goal_tf_mat = self.pose2tf_mat(self.goals[self.current_goal_index])
        goal_in_robot_coordinates = self.inverse_tf_mat(robot_tf_mat) @ goal_tf_mat # Por qué era un arroba para multiplicar matrices no?
        return self.tf_mat2pose(goal_in_robot_coordinates)

    # Se puede mejorar considerando los vt y wt anteriores
    def create_vt_and_wt(self):
        for vt in np.arange(0, 1.0, 0.1):
            for wt in np.arange(-1.4, 1.4, 0.05):
                yield vt, wt
