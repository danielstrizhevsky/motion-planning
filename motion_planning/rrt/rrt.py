from matplotlib import pyplot as plt
import random

from rrt_base import RRTBase
from motion_planning.config.rrt_config import (
    AGENT_START_POSITION,
    GOAL_POSITION,
    BOTTOM_BOUND,
    RIGHT_BOUND,
    VIS_PAUSE_LENGTH,
)


class RRT(RRTBase):
    def step(self, visualize: bool = True) -> None:
        random_point = random.random() * RIGHT_BOUND, random.random() * BOTTOM_BOUND
        nearest = self.nearest_neighbor(random_point)
        new_point = self.get_new_point(nearest, random_point)
        if new_point is None:
            return
        self.edges[nearest].append(new_point)
        self.parents[new_point] = nearest
        self.points.append(new_point)

        if visualize:
            line = plt.plot(
                [nearest[0], new_point[0]],
                [nearest[1], new_point[1]],
            )[0]
            self.edges_to_lines[(nearest, new_point)] = line
            plt.pause(VIS_PAUSE_LENGTH)


def run_rrt(num_steps: int) -> None:
    rrt = RRT(AGENT_START_POSITION, GOAL_POSITION)
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
    run_rrt(1000)
