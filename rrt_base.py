from collections import defaultdict
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from math import log2
import random
import time

from rrt_config import (
    AGENT_SPEED,
    BOTTOM_BOUND,
    DELTA,
    GAMMA_RRT_STAR,
    OBSTACLES,
    RIGHT_BOUND,
    VIS_PAUSE_LENGTH,
)


class RRTBase:
    def __init__(self, start, goal):
        self.points = [start]
        self.parents = {}
        self.edges = defaultdict(list)
        self.goal = goal
        self.agent_pos = start

        # For visualization
        self.edges_to_lines = {}
        self.agent_viz = None

    def init_plot(self):
        plt.xlim([0, RIGHT_BOUND])
        plt.ylim([0, BOTTOM_BOUND])
        plt.gca().set_aspect("equal")
        for obstacle in OBSTACLES:
            plt.gca().add_patch(Rectangle(*obstacle))
        plt.gca().add_patch(
            Rectangle(
                self.goal[0],
                self.goal[1][0] - self.goal[0][0],
                self.goal[1][1] - self.goal[0][1],
                color="green",
            )
        )
        self.agent_viz = plt.plot(10, 10, "ro", zorder=4)[0]

    def _is_in_goal(self, point):
        return (
            self.goal[0][0] <= point[0] <= self.goal[1][0]
            and self.goal[0][1] <= point[1] <= self.goal[1][1]
        )

    def _get_cost(self, point):
        cost = 0
        while point in self.parents:
            parent = self.parents[point]
            cost += self._calculate_distance(point, parent)
            point = parent
        return cost

    def _obstacle_free(self, point):
        """
        return False if point collides with an obstacle
        """
        for obstacle in OBSTACLES:
            left, top = obstacle[0]
            right, bottom = left + obstacle[1], top + obstacle[2]
            if left <= point[0] <= right and top <= point[1] <= bottom:
                return False
        return True

    def best_path_to_goal(self):
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

    def move_towards_point(self, path, to_travel=AGENT_SPEED):
        if not path:
            return
        point = path[-1]
        distance = self._calculate_distance(self.agent_pos, point)
        if distance <= to_travel:
            self.agent_pos = point
            path.pop()
            self.move_towards_point(path, to_travel - distance)
        else:
            dist_x = point[0] - self.agent_pos[0]
            dist_y = point[1] - self.agent_pos[1]
            new_point = (
                self.agent_pos[0] + to_travel / distance * dist_x,
                self.agent_pos[1] + to_travel / distance * dist_y,
            )
            self.agent_pos = new_point

    def _calculate_distance(self, a, b):
        return ((b[1] - a[1]) ** 2 + (b[0] - a[0]) ** 2) ** 0.5

    def nearest_neighbor(self, new_point):
        return min(self.points, key=lambda x: self._calculate_distance(new_point, x))

    def get_new_point(self, source, destination):
        """
        Returns the new point distance DELTA from the source to the destination
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

    def highlight_path_to_goal(self):
        path = self.best_path_to_goal()
        for i in range(len(path) - 1):
            start, end = path[i], path[i + 1]
            line = self.edges_to_lines[(start, end)]
            line.set_color("black")
            line.set_linewidth(2)
            line.set_zorder(3)
            plt.pause(VIS_PAUSE_LENGTH)

