import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from typing import Tuple
import numpy as np
from custom_types import Map, Point


COLORS = {
    "darkblue": "#143049",
    "twblue": "#00649C",
    "lightblue": "#8DA3B3",
    "lightgrey": "#CBC0D5",
    "twgrey": "#72777A"
}


def plot_map(ax: Axes, map: Map):
    """
    Plot the map in world coordinates.

    Args:
        ax (Axes): matplotlib axes object
        map (Map): map object containing the map data
    """
    walls_xs = []
    walls_ys = []
    points_xs = []
    points_ys = []
    for point in map.map.flatten():
        if point.wall:
            walls_xs.append(point.x)
            walls_ys.append(point.y)
        else:
            points_xs.append(point.x)
            points_ys.append(point.y)
    ax.scatter(walls_xs, walls_ys, c=COLORS["darkblue"], alpha=1.0, s=6**2, label="Walls")
    ax.scatter(points_xs, points_ys, c=COLORS["twgrey"], alpha=0.08, s=6**2, label="Unobstructed Space")


def plot_laser_scan(ax: Axes, laser_scan: np.ndarray):
    laser_scan_xs = []
    laser_scan_ys = []
    for point in laser_scan:
        laser_scan_xs.append(point.x)
        laser_scan_ys.append(point.y)
    ax.scatter(laser_scan_xs, laser_scan_ys, c="r", label="Laser Scan")


def plot_robot_position(ax: Axes, robot_position: Point):
    """
    Plot the robot position in world coordinates.

    Args:
        ax (Axes): matplotlib axes object
        robot_position (Point): robot position in world coordinates
    """
    ax.scatter([robot_position.x], [robot_position.y], c=COLORS["twblue"], s=15**2, label="Scan Center")


def plot_global_plan(ax: Axes, global_path: np.ndarray):
    """
    Plot the global path in world coordinates.

    Args:
        ax (Axes): matplotlib axes object
        global_path (np.ndarray): global path as an array of points
    """
    path_xs = []
    path_ys = []
    for point in global_path:
        path_xs.append(point.x)
        path_ys.append(point.y)
    ax.plot(path_xs, path_ys, "-o", c=COLORS["lightblue"], label="Global Path")


def plot_map_with_initial_position(map: Map, laser_scan: np.ndarray, robot_position: Point):
    """
    Plot the map and laser scan data in world coordinates.

    Args:
        map (Map): map object containing the map data
        laser_scan (np.ndarray): laser scan data
        robot_position (Point): robot position in world coordinates
    """
    fig, ax = plt.subplots()
    fig.set_size_inches(8, 8)
    
    plot_map(ax, map)
    plot_laser_scan(ax, laser_scan)
    plot_robot_position(ax, robot_position)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Map and Laser Scan Data Transformed into World Coordinates")
    ax.set_xticks(np.arange(-1, 5, 1))
    ax.set_yticks(np.arange(-1, 5, 1))
    ax.set_axisbelow(True)
    ax.grid()
    ax.legend()

    plt.show()


def plot_map_with_global_plan(map: Map, laser_scan: np.ndarray, robot_position: Point, global_path: np.ndarray):
    """
    Plot the map and laser scan data in world coordinates.

    Args:
        map (Map): map object containing the map data
        laser_scan (np.ndarray): laser scan data
        robot_position (Tuple[float, float]): robot position in world coordinates
    """
    fig, ax = plt.subplots()
    fig.set_size_inches(8, 8)
    
    plot_map(ax, map)
    plot_laser_scan(ax, laser_scan)
    plot_robot_position(ax, robot_position)
    plot_global_plan(ax, global_path)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Found Path from Robot Position to Goal Position")
    ax.set_xticks(np.arange(-1, 5, 1))
    ax.set_yticks(np.arange(-1, 5, 1))
    ax.set_axisbelow(True)
    ax.grid()
    ax.legend()

    plt.show()
