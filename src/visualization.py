import matplotlib.pyplot as plt
import numpy as np
from custom_types import Map


COLORS = {
    "darkblue": "#143049",
    "twblue": "#00649C",
    "lightblue": "#8DA3B3",
    "lightgrey": "#CBC0D5",
    "twgrey": "#72777A"
}


def plot_map_and_laser_scan(map: Map, laser_scan: np.ndarray):
    """
    Plot the map and laser scan data in world coordinates.
    Code snippet obtained from the Project Cookbook and modified afterwards.

    Args:
        map (Map): map object containing the map data
        laser_scan (np.ndarray): laser scan data
    """
    fig, ax = plt.subplots()
    fig.set_size_inches(8, 8)

    laser_scan_xs = []
    laser_scan_ys = []
    for point in laser_scan:
        laser_scan_xs.append(point.x)
        laser_scan_ys.append(point.y)
    ax.scatter(laser_scan_xs, laser_scan_ys, c="r", label="Laser Scan")
    
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
    ax.scatter([0], [0], c=COLORS["twblue"], s=15**2, label="Scan Center")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Map and Laser Scan Data Transformed into World Coordinates")
    ax.set_xticks(np.arange(-1, 5, 1))
    ax.set_yticks(np.arange(-1, 5, 1))
    ax.set_axisbelow(True)
    ax.grid()
    ax.legend()

    plt.show()
