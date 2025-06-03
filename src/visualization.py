import matplotlib.pyplot as plt
import numpy as np
from custom_types import Map, Point


COLORS = {
    "darkblue": "#143049",
    "twblue": "#00649C",
    "lightblue": "#8DA3B3",
    "lightgrey": "#CBC0D5",
    "twgrey": "#72777A"
}


class Plotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()

    def with_size(self, width: float, height: float):
        """
        Set the figure size.

        Args:
            width (float): width of the figure, in inches
            height (float): height of the figure, in inches
        """
        self.fig.set_size_inches(width, height)
        return self

    def with_map(self, map: Map):
        """
        Plot the map in world coordinates.

        Args:
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
        self.ax.scatter(walls_xs, walls_ys, c=COLORS["darkblue"], alpha=1.0, s=6**2, label="Walls")
        self.ax.scatter(points_xs, points_ys, c=COLORS["twgrey"], alpha=0.08, s=6**2, label="Unobstructed Space")
        return self

    def with_costmap(self, map: Map):
        """
        Plot the costmap in world coordinates.

        Args:
            map (Map): map object containing the costmap data
        """
        xs = []
        ys = []
        colors = []
        for point in map.costmap.flatten():
            xs.append(point.x)
            ys.append(point.y)
            colors.append(point.cost)
        self.ax.scatter(xs, ys, c=colors, cmap="gray_r", alpha=1.0, s=6**2)
        return self

    def with_laser_scan(self, laser_scan: np.ndarray):
        """
        Plot the laser scan data in world coordinates.

        Args:
            laser_scan (np.ndarray): laser scan data
        """
        laser_scan_xs = []
        laser_scan_ys = []
        for point in laser_scan:
            laser_scan_xs.append(point.x)
            laser_scan_ys.append(point.y)
        self.ax.scatter(laser_scan_xs, laser_scan_ys, c="r", label="Laser Scan")
        return self

    def with_robot(self, robot_position: Point):
        """
        Plot the robot position in world coordinates.

        Args:
            robot_position (Point): robot position in world coordinates
        """
        self.ax.scatter([robot_position.x], [robot_position.y], c=COLORS["twblue"], s=15**2, label="Scan Center")
        return self
        
    def with_global_plan(self, global_path: np.ndarray):
        """
        Plot the global path in world coordinates.

        Args:
            global_path (np.ndarray): global path as an array of points
        """
        path_xs = []
        path_ys = []
        for point in global_path:
            path_xs.append(point.x)
            path_ys.append(point.y)
        self.ax.plot(path_xs, path_ys, "-o", c=COLORS["lightblue"], label="Global Path")
        return self

    def with_title(self, title: str):
        """
        Set the title of the plot.

        Args:
            title (str): title of the plot
        """
        self.ax.set_title(title)
        return self
        
    def with_labels(self, xlabel: str, ylabel: str):
        """
        Set the labels of the plot.

        Args:
            xlabel (str): x-axis label
            ylabel (str): y-axis label
        """
        if xlabel is not None:
            self.ax.set_xlabel(xlabel)
        if ylabel is not None:
            self.ax.set_ylabel(ylabel)
        return self

    def with_ticks(self, xticks: np.ndarray, yticks: np.ndarray):
        """
        Set the ticks of the plot.

        Args:
            xticks (np.ndarray): x-axis ticks
            yticks (np.ndarray): y-axis ticks
        """
        if xticks is not None:
            self.ax.set_xticks(xticks)
        if yticks is not None:
            self.ax.set_yticks(yticks)
        return self

    def with_grid(self):
        """
        Set the grid of the plot.
        """
        self.ax.grid()
        return self

    def show(self):
        """
        Show the plot.
        """
        self.ax.legend()
        plt.show()
