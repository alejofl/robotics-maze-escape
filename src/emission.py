from abc import ABC, abstractmethod
import rospy
from typing import Any, List
from custom_types import Point
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped as RosPoseStamped


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
