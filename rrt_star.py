from matplotlib import pyplot as plt
from math import log2
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
    def min_cost_neighbor(self, new_point, neighbors):
        return min(
            neighbors,
            key=lambda x: self._get_cost(x) + self._calculate_distance(x, new_point),
        )

    def near_neighbors(self, new_point):
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

    def neighbors_to_rewire(self, neighbors, new_point):
        resulting_neighbors = []
        for neighbor in neighbors:
            if self._get_cost(new_point) + self._calculate_distance(
                new_point, neighbor
            ) < self._get_cost(neighbor):
                resulting_neighbors.append(neighbor)
        return resulting_neighbors

    def rewire_neighbor_through_new_point(self, neighbor, new_point, visualize=True):
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
        nearby = self.near_neighbors(new_point) + [nearest]
        best_neighbor = self.min_cost_neighbor(new_point, nearby)
        self.edges[best_neighbor].append(new_point)
        self.parents[new_point] = best_neighbor
        self.points.append(new_point)
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
