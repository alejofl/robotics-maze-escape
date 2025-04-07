#!/usr/bin/python
import rospy
from localization import get_map, deserialize_map, get_laser_scan
from visualization import plot_map_and_laser_scan


if __name__ == "__main__":
    rospy.init_node("maze-escape")
    map_data = get_map()
    map = deserialize_map(map_data)
    laser_scan = get_laser_scan(map.resolution)
    plot_map_and_laser_scan(map, laser_scan)