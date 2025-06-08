#!/usr/bin/python
from local_planning import RobotMovement
import rospy
import numpy as np
from custom_types import Point, Pose
from localization import get_map, deserialize_map, get_laser_scan, predict_robot_position
from global_planning import GlobalPlanner, GOAL_POSITIONS
from visualization import Plotter
from emission import GlobalPlanEmmiter, RobotMovementEmitter, TrajectoryEmitter, GoalPoseEmitter


if __name__ == "__main__":
    rospy.init_node("maze_escape")
    global_planner_algorithm = rospy.get_param("/maze_escape/global_planner_algorithm")
    global_planner_heuristic = rospy.get_param("/maze_escape/global_planner_heuristic")
    enable_plotting = rospy.get_param('/maze_escape/enable_plotting')
    goal = rospy.get_param('/maze_escape/goal') - 1

    global_plan_emitter = GlobalPlanEmmiter()
    velocity_emitter = RobotMovementEmitter()
    trajectory_emitter = TrajectoryEmitter()
    goal_pose_emitter = GoalPoseEmitter()

    # Localization of the robot based on the map and laser scan data
    rospy.loginfo("Starting localization based on laser scan data...")
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
    
    # Global planning to find a path from the robot position to the goal position
    rospy.loginfo("Starting global planning...")
    goal_pose_emitter.emit(
        [map.map[x][y] for x, y in GOAL_POSITIONS[goal]]
    )
    global_path = GlobalPlanner \
        .get_planner(global_planner_algorithm, map, robot_position_idx, global_planner_heuristic) \
        .plan(GOAL_POSITIONS[goal])
    if enable_plotting:
        Plotter() \
            .with_size(8, 8) \
            .with_costmap(map) \
            .with_robot(robot_position) \
            .with_title("Costmap with Robot Position") \
            .with_labels("x [m]", "y [m]") \
            .with_ticks(np.arange(-1, 5, 1), np.arange(-1, 5, 1)) \
            .with_grid() \
            .show()
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

    # Local planning to move the robot along the global path
    rospy.loginfo("Starting local planning and robot movement...")
    RobotMovement(
        Pose(robot_position.x, robot_position.y, 0),
        global_path,
        0.4,
        5,
        velocity_emitter,
        trajectory_emitter
    ).run()
