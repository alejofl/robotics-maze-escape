from abc import ABC, abstractmethod
import rospy
from typing import Any, List
from custom_types import Point
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from geometry_msgs.msg import Twist


class Emitter(ABC):
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
        self.publisher.publish(message)


class GoalPoseEmitter(Emitter):
    def __init__(self):
        super().__init__("/maze_escape/goal_pose", RosPoseStamped)

    def emit(self, data: Any):
        data: Point = data
        message = RosPoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        message.pose.position.x = data[0]
        message.pose.position.y = data[1]
        message.pose.position.z = 0.0
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = data[2]
        self.publisher.publish(message)


class TrajectoryEmitter(Emitter):
    def __init__(self):
        super().__init__("/maze_escape/trajectory", RosPath)

    def emit(self, data: Any):
        data: List[Point] = data
        message = RosPath()

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        for point in data:
            pose = RosPoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            message.pose.position.x = point[0]
            message.pose.position.y = point[1]
            message.pose.position.z = 0.0
            message.pose.orientation.x = 0.0
            message.pose.orientation.y = 0.0
            message.pose.orientation.z = 0.0
            message.pose.orientation.w = point[2]
            message.poses.append(pose)

        self.publisher.publish(message)
