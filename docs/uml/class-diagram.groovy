@startuml
package "custom_types" {
    class Map {
        + width: int
        + height: int
        + origin: Tuple[float, float]
        + resolution: float
        + map: MapPoint[][]
        + costmap: CostmapMapPoint[][]
    }

    class Point {
        + x: float
        + y: float
    }

    class MapPoint extends Point {
        + wall: bool
    }

    class CostmapMapPoint extends Point {
        + cost: float
    }

    class Pose extends Point {
        + theta: float
    }

    Map *-- Point
}

package "emission" {
    abstract class Emitter {
        - publisher: rospy.Publisher
        + emit()
    }

    class GlobalPlanEmmiter implements Emitter {
    }

    class RobotMovementEmitter implements Emitter {
    }

    class GoalPoseEmitter implements Emitter {
    }

    class TrajectoryEmitter implements Emitter {
    }
}

package "global_planning" {
    entity Node {
        + x: int
        + y: int
        + parent: Node
    }

    abstract class GlobalPlanner {
        - map: Map
        - initial_position: Tuple[int, int]
        + {static} get_planner()
        - get_heuristic()
        + plan()
    }

    class DFSPlanner extends GlobalPlanner {
    }

    class BFSPlanner extends GlobalPlanner {
    }

    class AStarPlanner extends GlobalPlanner {
    }
}

package "local_planning" {
    class RobotMovement {
        - initial_pose: Pose
        - goals: Point[]
        - time_step: float
        - horizon: int
        - velocity_publisher: RobotMovementEmitter
        - trajectory_publisher: TrajectoryEmitter
        - localize_robot()
        - get_goal_in_robot_coordinates()
        - get_pose_in_world_coordinates()
        - create_vt_and_wt()
        - cost_function()
        - is_goal_reached()
        + run()
    }

    class ForwardKinematics {
        + forward_kinematics()
    }

    class PT2Block {
        + update()
    }

    class MatrixManipulation {
        + pose_to_tf_matrix()
        + tf_matrix_to_pose()
        + inverse_matrix()
    }
}

package "localization" {
    class Localization {
        + get_map()
        + deserialize_map()
        + get_laser_scan()
        + predict_robot_position()
    }
}

package "visualization" {
    class Plotter {
        - fig: plt.Figure
        - ax: plt.Axes
        + with_size()
        + with_map()
        + with_costmap()
        + with_laser_scan()
        + with_robot()
        + with_global_plan()
        + with_labels()
        + with_ticks()
        + with_grid()
        + show()
    }
}
@enduml