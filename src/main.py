#!/usr/bin/python
import rospy
import numpy as np
from custom_types import Point
from localization import get_map, deserialize_map, get_laser_scan, predict_robot_position
from global_planning import GlobalPlanner, GOAL_POSITIONS
from visualization import Plotter
from emission import GlobalPlanEmmiter


if __name__ == "__main__":
    rospy.init_node("maze_escape")
    global_planner_algorithm = rospy.get_param("/maze_escape/global_planner_algorithm")
    enable_plotting = rospy.get_param('/maze_escape/enable_plotting')
    goal = rospy.get_param('/maze_escape/goal') - 1

    global_plan_emitter = GlobalPlanEmmiter()

    map_data = get_map()
    map = deserialize_map(map_data)
    laser_scan = get_laser_scan(map.resolution)
    if enable_plotting:
        Plotter() \
            .with_size(8, 8) \
            .with_map(map) \
            .with_laser_scan(laser_scan) \
            .with_robot(Point(0, 0)) \
            .with_title("Map and Laser Scan Data Transformed into World Coordinates") \
            .with_labels("x [m]", "y [m]") \
            .with_ticks(np.arange(-1, 5, 1), np.arange(-1, 5, 1)) \
            .with_grid() \
            .show()
    robot_position_idx, robot_position, laser_scan  = predict_robot_position(map, laser_scan)
    global_path = GlobalPlanner \
        .get_planner(global_planner_algorithm, map, robot_position_idx) \
        .plan(GOAL_POSITIONS[goal])
    if enable_plotting:
        Plotter() \
            .with_size(8, 8) \
            .with_map(map) \
            .with_robot(robot_position) \
            .with_global_plan(global_path) \
            .with_title("Found Path from Robot Position to Goal Position") \
            .with_labels("x [m]", "y [m]") \
            .with_ticks(np.arange(-1, 5, 1), np.arange(-1, 5, 1)) \
            .with_grid() \
            .show()
    global_plan_emitter.emit(global_path)
