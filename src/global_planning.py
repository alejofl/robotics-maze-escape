import numpy as np
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from custom_types import Map
from typing import Tuple, List
from collections import deque
from queue import PriorityQueue


GOAL_POSITIONS = [
    [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7)],
    [(26, 19), (26, 20), (26, 21), (26, 22), (26, 23), (26, 24), (26, 25)]
]


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

    def manhattan_distance(self, goal_position: List[Tuple[int, int]]) -> float:
        """
        Calculate the Manhattan distance from the current node to the goal position.
        
        Args:
            goal_position (List[Tuple[int, int]]): The goal position to reach, in map indices.
        
        Returns:
            float: The Manhattan distance to the goal position.
        """
        return np.min([np.abs(self.x - x) + np.abs(self.y - y) for x, y in goal_position])

    def euclidean_distance(self, goal_position: List[Tuple[int, int]]) -> float:
        """
        Calculate the Euclidean distance from the current node to the goal position.
        
        Args:
            goal_position (List[Tuple[int, int]]): The goal position to reach, in map indices.
        
        Returns:
            float: The Euclidean distance to the goal position.
        """
        return np.min([np.sqrt((self.x - x) ** 2 + (self.y - y) ** 2) for x, y in goal_position])


@dataclass(order=True, frozen=True)
class PrioritizedNode:
    priority: float
    cost: float
    node: Node = field(compare=False)


class GlobalPlanner(ABC):
    def __init__(self, map: Map, initial_position: Tuple[int, int], heuristic: str = None):
        self.map = map
        self.initial_position = initial_position
        self.initial_node = Node(*initial_position, None)
        self.heuristic = self.get_heuristic(heuristic)

    @abstractmethod
    def plan(self, goal_position: List[Tuple[int, int]] = GOAL_POSITIONS[0]) -> np.ndarray:
        """
        Plan a path from the initial position to the goal position.
        
        Args:
            goal_position (List[Tuple[int, int]]): The goal position to reach, in map indices. Default is the first goal position.
        
        Returns:
            np.ndarray[Point]: The planned path as an array of points.
        """
        pass

    @staticmethod
    def get_planner(planner_algorithm: str, map: Map, initial_position: Tuple[float, float], heuristic: str = None) -> 'GlobalPlanner':
        """
        Factory method to get the appropriate planner based on the planner type.
        
        Args:
            planner_algorithm (str): The type of planner to create.
            map (Map): The map to use for planning.
            initial_position (Tuple[int, int]): The initial position of the robot, in map indices.
            heuristic (str): The heuristic function to use for path planning. Only needed for A* planner.
        
        Returns:
            GlobalPlanner: An instance of the specified planner type.
        """
        if planner_algorithm.lower() == "dfs":
            return DFSPlanner(map, initial_position, None)
        elif planner_algorithm.lower() == "bfs":
            return BFSPlanner(map, initial_position, None)
        elif planner_algorithm.lower() == "astar":
            return AStarPlanner(map, initial_position, heuristic)
        elif planner_algorithm.lower() == "dijkstra":
            pass
        else:
            raise ValueError(f"Unknown planner algorithm: {planner_algorithm}")
        
    def get_unused_goal_positions(self, goal_position: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Get the unused goal positions from the list of goal positions.
        
        Args:
            goal_positions (List[Tuple[int, int]]): The used goal position.
        
        Returns:
            List[Tuple[int, int]]: The list of unused goal positions.
        """
        unused_goal_positions = []
        for positions in GOAL_POSITIONS:
            if positions != goal_position:
                for position in positions:
                        unused_goal_positions.append(position)
        return unused_goal_positions

    def get_heuristic(self, heuristic: str) -> callable:
        """
        Get the heuristic function to use for path planning.
        
        Returns:
            Callable: The heuristic function.
        """
        if heuristic is None:
            return None
        
        if heuristic.lower() == "manhattan":
            return lambda node, goal_position: node.manhattan_distance(goal_position)
        elif heuristic.lower() == "euclidean":
            return lambda node, goal_position: node.euclidean_distance(goal_position)
        else:
            raise ValueError(f"Unknown heuristic: {heuristic}")


class BFSPlanner(GlobalPlanner):
    def plan(self, goal_position: List[Tuple[int, int]] = GOAL_POSITIONS[0]) -> np.ndarray:
        unused_goal_positions = self.get_unused_goal_positions(goal_position)
        visited_nodes: set[Node] = set()
        queue: deque[Node] = deque()
        queue.append(self.initial_node)

        while queue:
            curr = queue.popleft()
            if curr in visited_nodes:
                continue
            visited_nodes.add(curr)

            if (curr.x, curr.y) in goal_position:
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
                    if not self.map.map[x][y].wall and (x, y) not in unused_goal_positions:
                        queue.append(Node(x, y, curr))

        raise RuntimeError("No path found to the goal position.")


class DFSPlanner(GlobalPlanner):
    def plan(self, goal_position: List[Tuple[int, int]] = GOAL_POSITIONS[0]) -> np.ndarray:
        unused_goal_positions = self.get_unused_goal_positions(goal_position)
        visited_nodes: set[Node] = set()
        queue: deque[Node] = deque()
        queue.append(self.initial_node)

        while queue:
            curr = queue.pop()
            if curr in visited_nodes:
                continue
            visited_nodes.add(curr)

            if (curr.x, curr.y) in goal_position:
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
                    if not self.map.map[x][y].wall and (x, y) not in unused_goal_positions:
                        queue.append(Node(x, y, curr))

        raise RuntimeError("No path found to the goal position.")


class AStarPlanner(GlobalPlanner):
    def __init__(self, map: Map, initial_position: Tuple[int, int], heuristic: str = None):
        super().__init__(map, initial_position, heuristic)
        if self.heuristic is None:
            raise ValueError("A* planner requires a heuristic function.")

    def plan(self, goal_position: List[Tuple[int, int]] = GOAL_POSITIONS[0]) -> np.ndarray:
        unused_goal_positions = self.get_unused_goal_positions(goal_position)
        visited_nodes: set[Node] = set()
        queue: PriorityQueue[PrioritizedNode] = PriorityQueue()
        queue.put(PrioritizedNode(0, 0, self.initial_node))
        
        while queue:
            prioritized_node = queue.get()
            curr = prioritized_node.node
            if curr in visited_nodes:
                continue
            visited_nodes.add(curr)

            if (curr.x, curr.y) in goal_position:
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
                    if not self.map.map[x][y].wall and (x, y) not in unused_goal_positions:
                        next_node = Node(x, y, curr)
                        queue.put(PrioritizedNode(
                            prioritized_node.cost + 1 + self.heuristic(next_node, goal_position),
                            prioritized_node.cost + 1,
                            next_node
                        ))

        raise RuntimeError("No path found to the goal position.")
