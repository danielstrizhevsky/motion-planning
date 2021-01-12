from matplotlib import pyplot as plt
import random

from rrt_base import RRTBase
from rrt_config import (
    BOTTOM_BOUND,
    RIGHT_BOUND,
    VIS_PAUSE_LENGTH,
)

class RRT(RRTBase):
    def step(self, visualize=True):
        random_point = (
            random.random() * RIGHT_BOUND, random.random() * BOTTOM_BOUND
        )
        nearest = self.nearest_neighbor(random_point)
        new_point = self.get_new_point(nearest, random_point)
        if new_point is None:
            return
        self.edges[nearest].append(new_point)
        self.parents[new_point] = nearest
        self.points.append(new_point)

        if visualize:
            line, = plt.plot(
                [nearest[0], new_point[0]],
                [nearest[1], new_point[1]],
            )
            self.edges_to_lines[(nearest, new_point)] = line
            plt.pause(VIS_PAUSE_LENGTH)


def run_rrt(num_steps):
    rrt = RRT(
        (10, 10),
        ((85, 85), (95, 95)),
    )
    rrt.init_plot()

    for i in range(num_steps):
        if i % 100 == 0:
            print(i, "steps completed.")
        rrt.step()
    rrt.highlight_path_to_goal()
    path = rrt.best_path_to_goal()[::-1]
    while path:
        rrt.move_towards_point(path)
        rrt.agent_viz.set_xdata(rrt.agent_pos[0])
        rrt.agent_viz.set_ydata(rrt.agent_pos[1])
        plt.pause(VIS_PAUSE_LENGTH)
    plt.show()


if __name__ == "__main__":
    run_rrt(1000)
