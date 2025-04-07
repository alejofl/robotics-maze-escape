from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class Map:
    width: int
    height: int
    origin: Tuple[float, float]
    resolution: float
    map: np.ndarray


@dataclass
class Point:
    x: float
    y: float


@dataclass
class MapPoint(Point):
    wall: bool
