from dataclasses import dataclass
import numpy as np


@dataclass
class Map:
    width: int
    height: int
    resolution: float
    map: np.ndarray


@dataclass
class Point:
    x: float
    y: float


@dataclass
class MapPoint(Point):
    wall: bool
