#!/usr/bin/python
import rospy
from custom_types import Point
from localization import get_map, deserialize_map, get_laser_scan, predict_robot_position
from global_planning import GlobalPlanner
from visualization import plot_map_with_initial_position, plot_map_with_global_plan


if __name__ == "__main__":
    rospy.init_node("maze-escape")
    map_data = get_map()
    map = deserialize_map(map_data)
    laser_scan = get_laser_scan(map.resolution)
    # plot_map_with_initial_position(map, laser_scan, Point(0, 0))
    robot_position_idx, robot_position, laser_scan  = predict_robot_position(map, laser_scan)
    global_path = GlobalPlanner.get_planner("bfs", map, robot_position_idx).plan()
    # plot_map_with_global_plan(map, laser_scan, robot_position, global_path)