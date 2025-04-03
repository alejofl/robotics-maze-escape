#!/usr/bin/python
import rospy
from localization import get_map, deserialize_map


if __name__ == "__main__":
    rospy.init_node("maze-escape")
    map_data = get_map()
    map = deserialize_map(map_data)
    print(f"Map: {map}")
