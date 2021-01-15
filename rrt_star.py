from matplotlib import pyplot as plt
from math import log2
from typing import List, Tuple
import random

from rrt_base import RRTBase
from rrt_config import (
    DELTA,
    GAMMA_RRT_STAR,
    RIGHT_BOUND,
    BOTTOM_BOUND,
    VIS_PAUSE_LENGTH,
    EPSILON,
)


class RRTStar(RRTBase):
    def min_cost_neighbor(
        self, new_point: Tuple[float, float], neighbors: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Given a new point and a list of neighbors, return the one that minimizes
        cost(neighbor) + distance(neighbor, new_point)

        new_point: (x, y) tuple describing the new point
        neighbors: list of points from which we want to choose
        return: The point in neighbors that minimizes distance through it to new_point
        """
        return min(
            neighbors,
            key=lambda neighbor: (
                self._get_cost(neighbor) + self._calculate_distance(neighbor, new_point)
            ),
        )

    def near_neighbors(
        self, new_point: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Finds all points in the tree that are within
        GAMMA_RRT_STAR * sqrt(log_2(number of points) / number of points)
        distance from the new point.

        new_point: (x, y) tuple describing the new point
        return: List of points in the RRT this distance from the new point
        """
        if len(self.points) == 1:
            return self.points
        neighbors = []
        distance_cutoff = min(
            GAMMA_RRT_STAR * (log2(len(self.points)) / len(self.points)) ** 0.5, DELTA
        )
        for point in self.points:
            dist = self._calculate_distance(new_point, point)
            if dist - EPSILON <= distance_cutoff:
                neighbors.append(point)
        return neighbors

    def neighbors_to_rewire(
        self, neighbors: List[Tuple[float, float]], new_point: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Return neighbors that should be rewired given a list of neighbors and a new
        point to rewire through. Neighbors should be rewired if:
        cost(new_point) + distance(new_point, neighbor) < cost(neighbor)

        neighbors: list of points from which we choose a subset to rewire
        new_point: (x, y) tuple describing the new point
        return: a list of points describing the points from neighbors that should
        be rewired.
        """
        resulting_neighbors = []
        for neighbor in neighbors:
            if self._get_cost(new_point) + self._calculate_distance(
                new_point, neighbor
            ) < self._get_cost(neighbor):
                resulting_neighbors.append(neighbor)
        return resulting_neighbors

    def rewire_neighbor_through_new_point(
        self,
        neighbor: Tuple[float, float],
        new_point: Tuple[float, float],
        visualize: bool = True,
    ) -> None:
        """
        Given a neighbor and a new point (both points in the RRT),
        rewire the neighbor through the new point. The new point becomes
        its new parent, and we remove the edge between the neigbhbor and
        its previous parent.

        neighbor: (x, y) tuple representing point to rewire through new point
        new_point: (x, y) tuple representing new parent of the neighbor
        """
        parent = self.parents[neighbor]
        self.parents[neighbor] = new_point
        self.edges[parent].remove(neighbor)
        self.edges[new_point].append(neighbor)

        if visualize:
            line = plt.plot(
                [new_point[0], neighbor[0]],
                [new_point[1], neighbor[1]],
            )[0]
            self.edges_to_lines[(new_point, neighbor)] = line
            self.edges_to_lines[(parent, neighbor)].remove()
            del self.edges_to_lines[(parent, neighbor)]

    def step(self, visualize=True):
        random_point = random.random() * RIGHT_BOUND, random.random() * BOTTOM_BOUND
        nearest = self.nearest_neighbor(random_point)
        new_point = self.get_new_point(nearest, random_point)
        if new_point is None:
            return
        # get nearby neighbors to the new point, including nearest
        nearby = self.near_neighbors(new_point) + [nearest]
        best_neighbor = self.min_cost_neighbor(new_point, nearby)
        self.edges[best_neighbor].append(new_point)
        self.parents[new_point] = best_neighbor
        self.points.append(new_point)
        # Rewire neighbors that would have a better cost if they went
        # through our new point instead.
        for neighbor in self.neighbors_to_rewire(nearby[:-1], new_point):
            self.rewire_neighbor_through_new_point(
                neighbor, new_point, visualize=visualize
            )

        if visualize:
            line = plt.plot(
                [best_neighbor[0], new_point[0]],
                [best_neighbor[1], new_point[1]],
            )[0]
            self.edges_to_lines[(best_neighbor, new_point)] = line
            plt.pause(VIS_PAUSE_LENGTH)
        return self._is_in_goal(new_point)


def run_rrt_star(num_steps):
    rrt = RRTStar(
        (10, 10),
        ((85, 85), (95, 95)),
    )
    rrt.init_plot()

    for i in range(num_steps):
        if i % 100 == 0:
            print(i, "steps completed.")
        rrt.step()
    rrt.redraw_path_to_goal()
    path = rrt.best_path_to_goal()[::-1]
    rrt.move_along_path_until_done(path)
    plt.show()


if __name__ == "__main__":
    run_rrt_star(2000)
