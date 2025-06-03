from abc import ABC, abstractmethod
import rospy
from typing import Any, List
from custom_types import Point, Pose
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped as RosPoseStamped, Twist


class Emitter(ABC):
    """
    Abstract base class for emitting messages to ROS topics.
    This class provides a template for creating emitters that publish messages
    to specific ROS topics with a defined message type.
    It requires subclasses to implement the `emit` method, which is responsible
    for publishing the data to the specified topic.
    """
    def __init__(self, topic: str, message_type: Any):
        self.publisher = rospy.Publisher(topic, message_type, queue_size=10)

    @abstractmethod
    def emit(self, data: Any):
        """
        Emit data.
        
        Args:
            data (Any): The data to emit.
        """
        pass


class GlobalPlanEmmiter(Emitter):
    """
    Emitter for global plan messages.
    This class inherits from the Emitter base class and is responsible for
    publishing global path messages to the ROS topic `/maze_escape/global_plan`.
    It formats the data as a `nav_msgs/Path` message.
    """
    def __init__(self):
        super().__init__("/maze_escape/global_plan", RosPath)

    def emit(self, data: Any):
        data: List[Point] = data
        message = RosPath()

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        for point in data:
            pose = RosPoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0
            message.poses.append(pose)

        self.publisher.publish(message)


class RobotMovementEmitter(Emitter):
    """
    Emitter for robot movement commands.
    This class inherits from the Emitter base class and is responsible for
    publishing movement commands to the ROS topic `/cmd_vel`.
    It formats the data as a `geometry_msgs/Twist` message.
    """
    def __init__(self):
        super().__init__("/cmd_vel", Twist)

    def emit(self, data: Any):
        control: List[float] = data
        message = Twist()
        message.linear.x = control[0]
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = control[1]  # Perpendicular to the plane X-Y

        self.publisher.publish(message)


class GoalPoseEmitter(Emitter):
    """
    Emitter for goal pose messages.
    This class inherits from the Emitter base class and is responsible for
    publishing goal pose messages to the ROS topic `/maze_escape/goal_pose`.
    It formats the data as a `geometry_msgs/PoseStamped` message.
    """
    def __init__(self):
        super().__init__("/maze_escape/goal_pose", RosPath)

    def emit(self, data: Any):
        data: List[Point] = data
        message = RosPath()

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        for point in data:
            pose = RosPoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0
            message.poses.append(pose)

        self.publisher.publish(message)


class TrajectoryEmitter(Emitter):
    """
    Emitter for trajectory messages.
    This class inherits from the Emitter base class and is responsible for 
    publishing trajectory messages to the ROS topic `/maze_escape/trajectory`.
    It formats the data as a `nav_msgs/Path` message.
    """
    def __init__(self):
        super().__init__("/maze_escape/trajectory", RosPath)

    def emit(self, data: Any):
        data: List[Pose] = data
        message = RosPath()

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        for point in data:
            pose = RosPoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = point.theta
            message.poses.append(pose)

        self.publisher.publish(message)
