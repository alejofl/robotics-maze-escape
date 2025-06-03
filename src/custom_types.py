from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class Map:
    """
    Represents a map with its dimensions, origin, resolution, and the map data itself.
    
    Args:
        width (int): The width of the map in pixels.
        height (int): The height of the map in pixels.
        origin (Tuple[float, float]): The origin coordinates of the map in world coordinates (x, y).
        resolution (float): The resolution of the map in pixels per meter.
        map (np.ndarray): The map data as a 2D numpy array, where each element represents a cell in the map.
        costmap (np.ndarray, optional): An optional costmap for the map, used for obstacle avoidance. Created using a gaussian filter.
    """
    width: int
    height: int
    origin: Tuple[float, float]
    resolution: float
    map: np.ndarray
    costmap: np.ndarray = None


@dataclass
class Point:
    """
    Represents a point in 2D space with x and y coordinates.
    
    Args:
        x (float): The x coordinate of the point.
        y (float): The y coordinate of the point.
    """
    x: float
    y: float


@dataclass
class MapPoint(Point):
    """
    Represents a point in the map with x and y coordinates, and a wall indicator.
    
    Args:
        wall (bool): Indicates whether the point is a wall or not.
    """
    wall: bool


@dataclass
class CostmapMapPoint(Point):
    """
    Represents a point in the costmap with x and y coordinates, and a cost value.
    
    Args:
        cost (float): The cost associated with the point.
    """
    cost: float


@dataclass
class Pose(Point):
    """
    Represents a pose in 2D space with x, y coordinates and orientation theta.
    
    Args:
        theta (float): The orientation of the pose in radians.
    """
    theta: float
