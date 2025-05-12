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
    global_planner_heuristic = rospy.get_param("/maze_escape/global_planner_heuristic")
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
        .get_planner(global_planner_algorithm, map, robot_position_idx, global_planner_heuristic) \
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
        
    # Podemos hacer que el robot reciba los goals (RobotMovement), en lugar de emitirlos en un topic
    global_plan_emitter.emit(global_path)

    # DONE Para la primera iteración, tenemos la posición del robot. A partir de ahí tenemos que irla calculando con cada movimiento
    # DONE Hacer un poco de voodoo matricial para tener el goal en función de la posición del robot. Pero está todo claro en el cookbook
    # DONE Crear varias (vt, wt) para simular para dónde se mueve el robot (Considerar usar el par (vt, wt) anterior)
    # Hacer la simulación con lo de Kinematics y pasar los resultados por una función de costo
    # Quedarse con el que tenga menor costo
    # Hacer un publish de la velocidad y la rotación
    # Repeat hasta llegar al último goal, después de eso hacer un publish de (0, 0) para que se frene

