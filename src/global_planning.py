import numpy as np
from dataclasses import dataclass
from abc import ABC, abstractmethod
from custom_types import Map, Point
from typing import Tuple
from collections import deque


GOAL_POSITIONS = [(0, 4), (26, 22)]


@dataclass(eq=False, frozen=True)
class Node:
    x: int
    y: int
    parent: 'Node' = None

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, value):
        if not isinstance(value, Node):
            return False
        return self.x == value.x and self.y == value.y


class GlobalPlanner(ABC):
    def __init__(self, map: Map, initial_position: Tuple[int, int]):
        self.map = map
        self.initial_position = initial_position
        self.initial_node = Node(*initial_position, None)

    @abstractmethod
    def plan(self, goal_position: Tuple[int, int] = GOAL_POSITIONS[0]) -> np.ndarray:
        """
        Plan a path from the initial position to the goal position.
        
        Args:
            goal_position (Tuple[int, int]): The goal position to reach, in map indices. Default is the first goal position.
        
        Returns:
            np.ndarray[Point]: The planned path as an array of points.
        """
        pass

    @staticmethod
    def get_planner(planner_algorithm: str, map: Map, initial_position: Tuple[float, float]) -> 'GlobalPlanner':
        """
        Factory method to get the appropriate planner based on the planner type.
        
        Args:
            planner_algorithm (str): The type of planner to create.
            map (Map): The map to use for planning.
            initial_position (Tuple[int, int]): The initial position of the robot, in map indices.
        
        Returns:
            GlobalPlanner: An instance of the specified planner type.
        """
        if planner_algorithm.lower() == "dfs":
            pass
        elif planner_algorithm.lower() == "bfs":
            return BFSPlanner(map, initial_position)
        elif planner_algorithm.lower() == "a*":
            pass
        elif planner_algorithm.lower() == "dijkstra":
            pass
        else:
            raise ValueError(f"Unknown planner algorithm: {planner_algorithm}")


class BFSPlanner(GlobalPlanner):
    def plan(self, goal_position: Tuple[int, int] = GOAL_POSITIONS[0]) -> np.ndarray:
        visited_nodes: set[Node] = set()
        queue: deque[Node] = deque()
        queue.append(self.initial_node)

        while queue:
            curr = queue.popleft()
            if curr in visited_nodes:
                continue
            visited_nodes.add(curr)
            print(curr.x, curr.y)

            if (curr.x, curr.y) == goal_position:
                path = []
                while curr:
                    path.append(self.map.map[curr.x][curr.y])
                    curr = curr.parent
                return np.flip(path)

            for x in np.arange(curr.x - 1, curr.x + 2):
                if x < 0 or x >= self.map.width:
                    continue
                for y in np.arange(curr.y - 1, curr.y + 2):
                    if y < 0 or y >= self.map.height or (x, y) == (curr.x, curr.y):
                        continue
                    if not self.map.map[x][y].wall:
                        queue.append(Node(x, y, curr))

        raise RuntimeError("No path found to the goal position.")
