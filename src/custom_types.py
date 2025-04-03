from dataclasses import dataclass
from typing import Tuple
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
    wall: bool