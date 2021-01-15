from abc import ABC, abstractmethod
from collections import defaultdict
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from math import log2
from typing import List, Tuple
import random
import time

from config.rrt_config import (
    AGENT_SPEED,
    BOTTOM_BOUND,
    DELTA,
    GAMMA_RRT_STAR,
    OBSTACLES,
    RIGHT_BOUND,
    VIS_PAUSE_LENGTH,
)


class RRTBase(ABC):
    def __init__(self, start, goal):
        self.points = [start]
        self.parents = {}
        self.edges = defaultdict(list)
        self.goal = goal
        self.agent_pos = start

        # For visualization
        self.edges_to_lines = {}
        self.lines_to_goal = []
        self.agent_viz = None
        self.goal_viz = None

    def init_plot(self) -> None:
        """
        Set bounds and draw obstacles, goal, and agent.
        """
        plt.xlim([0, RIGHT_BOUND])
        plt.ylim([0, BOTTOM_BOUND])
        plt.gca().set_aspect("equal")
        for obstacle in OBSTACLES:
            plt.gca().add_patch(Rectangle(*obstacle))
        self.goal_viz = plt.gca().add_patch(
            Rectangle(
                self.goal[0],
                self.goal[1][0] - self.goal[0][0],
                self.goal[1][1] - self.goal[0][1],
                color="#00FF00",
            )
        )
        self.agent_viz = plt.plot(10, 10, "ro", zorder=4)[0]

    @abstractmethod
    def step(self):
        """
        Perform one full step of adding a point to the RRT.
        Includes choosing a random point, steering towards it,
        rewiring if applicable, etc.
        """
        pass

    def _is_in_goal(self, point: Tuple[float, float]) -> bool:
        """
        Return whether the given point is within the boundaries
        of the goal.

        point: (x, y) tuple representing the point
        return: whether the point is in the goal
        """
        return (
            self.goal[0][0] <= point[0] <= self.goal[1][0]
            and self.goal[0][1] <= point[1] <= self.goal[1][1]
        )

    def _get_cost(self, point: Tuple[float, float]) -> float:
        """
        Return the cost from the root of a point in the tree.
        Cost is the sum of edge lengths from the root to this point.

        point: (x, y) tuple representing the point
        return: cost from the root to this point along the tree
        """
        cost = 0
        while point in self.parents:
            parent = self.parents[point]
            cost += self._calculate_distance(point, parent)
            point = parent
        return cost

    def _obstacle_free(self, point: Tuple[float, float]) -> float:
        """
        Return whether this point is free of obstacles

        point: (x, y) tuple representing the point
        return: False if the point collides with an obstacle, otherwise True

        TODO: more collision checks along an edge as opposed to just at one
        point
        """
        for obstacle in OBSTACLES:
            left, top = obstacle[0]
            right, bottom = left + obstacle[1], top + obstacle[2]
            if left <= point[0] <= right and top <= point[1] <= bottom:
                return False
        return True

    def best_path_to_goal(self) -> List[Tuple[float, float]]:
        """
        Return the lowest-cost path to the goal from the root of the RRT.

        return: the shortest path, comprised of a list of points
        in order from root to goal.
        """
        points_in_goal = [point for point in self.points if self._is_in_goal(point)]
        if not points_in_goal:
            return []
        cur_point = min(points_in_goal, key=lambda x: self._get_cost(x))
        print("Path cost:", self._get_cost(cur_point))
        path = [cur_point]
        while cur_point in self.parents:
            cur_point = self.parents[cur_point]
            path.append(cur_point)

        return path[::-1]

    def take_step_along_path(
        self, path: List[Tuple[float, float]], to_travel: float = AGENT_SPEED
    ) -> None:
        """
        Note: path is expected to be in reverse order for computational
        efficiency.
        TODO: use a queue to make this more straightforward

        Move the agent "to_travel" units along the path, updating the
        path to remove points as they're reached.

        path: list of point tuples in reverse order
        to_travel: distance left to travel, used for recursive calls if
        the next point is less than to_travel away from the agent
        """
        if not path:
            return
        point = path[-1]
        distance = self._calculate_distance(self.agent_pos, point)
        if distance <= to_travel:
            self.agent_pos = point
            path.pop()
            self.take_step_along_path(path, to_travel - distance)
        else:
            dist_x = point[0] - self.agent_pos[0]
            dist_y = point[1] - self.agent_pos[1]
            new_point = (
                self.agent_pos[0] + to_travel / distance * dist_x,
                self.agent_pos[1] + to_travel / distance * dist_y,
            )
            self.agent_pos = new_point

    def move_along_path_until_done(
        self,
        path: List[Tuple[float, float]],
        keep_stepping: bool = False,
        visualize: bool = True,
    ) -> None:
        """
        Keep moving the agent along the path until it's reached the end.

        path: list of point tuples in reverse order
        keep_stepping: Whether to call the RRT's step function while moving.
        Useful for anytime planning.
        visualize: Whether to call plt.pause()
        """
        while path:
            self.take_step_along_path(path)
            self.agent_viz.set_xdata(self.agent_pos[0])
            self.agent_viz.set_ydata(self.agent_pos[1])
            if keep_stepping:
                self.step()
            if visualize:
                plt.pause(VIS_PAUSE_LENGTH)

    def _calculate_distance(
        self, a: Tuple[float, float], b: Tuple[float, float]
    ) -> float:
        """
        Calculate the Euclidean distance between two points

        a: first (x, y) point
        b: second (x, y) point
        return: Euclidean distance between a and b
        """
        return ((b[1] - a[1]) ** 2 + (b[0] - a[0]) ** 2) ** 0.5

    def nearest_neighbor(self, new_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Return the closest point in the RRT to the given new point

        new_point: (x, y) tuple representing a new point
        return: (x, y) tuple of a point in the tree nearest to this point.
        """
        return min(self.points, key=lambda x: self._calculate_distance(new_point, x))

    def get_new_point(
        self, source: Tuple[float, float], destination: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Returns a new point distance DELTA from the source, towards the destination

        source: first (x, y) point
        destination: second (x, y) point to steer from source to

        return: a new (x, y) point distance DELTA closer to destination, from source
        """
        distance = self._calculate_distance(source, destination)
        if distance <= DELTA:
            if not self._obstacle_free(destination):
                return None
            return destination
        dist_x = destination[0] - source[0]
        dist_y = destination[1] - source[1]
        new_point = (
            source[0] + DELTA / distance * dist_x,
            source[1] + DELTA / distance * dist_y,
        )
        if not self._obstacle_free(new_point):
            return None
        return new_point

    def redraw_path_to_goal(self) -> None:
        """
        Remove the current path to goal from the visualization, and redraw
        a new one based on the current best path to the goal.
        """
        for line in self.lines_to_goal:
            line.remove()
        self.lines_to_goal = []
        path = self.best_path_to_goal()
        for i in range(len(path) - 1):
            start, end = path[i], path[i + 1]
            line = plt.plot(
                [start[0], end[0]],
                [start[1], end[1]],
            )[0]
            line.set_color("black")
            line.set_linewidth(2)
            line.set_zorder(3)
            self.lines_to_goal.append(line)
        plt.pause(VIS_PAUSE_LENGTH)
