from matplotlib.backend_bases import MouseEvent
from matplotlib import pyplot as plt
from math import log2
from typing import List, Tuple
import random

from rrt_star import RRTStar
from rrt_config import (
    ANYTIME_TRAJECTORY_LENGTH,
    DELTA,
    GAMMA_RRT_STAR,
    RIGHT_BOUND,
    BOTTOM_BOUND,
    VIS_PAUSE_LENGTH,
    EPSILON,
)


class AnytimeRRTStar(RRTStar):
    def __init__(self, start, goal):
        super().__init__(start, goal)
        self.lines_from_start = []
        self.lines_from_current = []

    def step_until_found_goal(self):
        """
        Keep running steps of the RRT until the goal is found.
        """
        found_goal = False
        while not found_goal:
            found_goal = self.step()

    def update_and_draw_current_trajectory(
        self, trajectory: List[Tuple[float, float]]
    ) -> None:
        """
        Draw edges in the current trajectory. Also update RRT's visualization
        of the trajectory from the start to include the most recent trajectory.
        That trajectory can be deleted when a new goal is chosen, but the current one
        remains drawn.

        trajectory: List of points to draw edges from and to update the RRT's
        internal tracker of drawn edges.
        """
        self.lines_from_start.extend(self.lines_from_current)
        self.lines_from_current = []
        for i in range(len(trajectory) - 1):
            start, end = trajectory[i], trajectory[i + 1]
            line = plt.plot(
                [start[0], end[0]],
                [start[1], end[1]],
            )[0]
            line.set_color("#00FF00")
            line.set_linewidth(2)
            line.set_zorder(3)
            self.lines_from_current.append(line)

    def prune_tree_and_set_new_root(
        self, trajectory: List[Tuple[float, float]]
    ) -> None:
        """
        Given a trajectory, set the new root to the end of the trajectory by
        deleting all branches of the tree that originate on that trajectory.

        trajectory: List of points from which to prune branches, so that the
        new root becomes the end of the trajectory.
        """
        new_root = trajectory[-1]
        stack = [trajectory[0]]
        while stack:
            cur = stack.pop()
            parent = self.parents.get(cur)
            if parent:
                del self.parents[cur]
                self.edges[parent].remove(cur)
                self.edges_to_lines[(parent, cur)].remove()
                del self.edges_to_lines[(parent, cur)]
            if cur != new_root:
                self.points.remove(cur)
                stack.extend(self.edges[cur])


def onclick(event: MouseEvent, rrt: AnytimeRRTStar) -> None:
    """
    Update the given rrt's goal to be centered at the x, y coordinate
    of the mouse click event.

    event: mouse click event with x and y data
    rrt: RRT whose goal to update
    """
    rrt.goal = (
        (event.xdata - 5, event.ydata - 5),
        (event.xdata + 5, event.ydata + 5),
    )
    rrt.goal_viz.set_xy((rrt.goal[0]))
    for line in rrt.lines_from_start:
        line.remove()
    rrt.lines_from_start = []
    rrt.redraw_path_to_goal()


def run_rrt_star():
    rrt = AnytimeRRTStar(
        (10, 10),
        ((85, 85), (95, 95)),
    )
    fig, _ = plt.subplots()
    fig.canvas.mpl_connect("button_press_event", lambda event: onclick(event, rrt))
    rrt.init_plot()

    while True:
        trajectory = rrt.best_path_to_goal()[:ANYTIME_TRAJECTORY_LENGTH]
        # Keep stepping the RRT until we find the goal.
        if not trajectory:
            rrt.step_until_found_goal()
        trajectory = rrt.best_path_to_goal()[:ANYTIME_TRAJECTORY_LENGTH]
        rrt.update_and_draw_current_trajectory(trajectory)
        # Once we've set the current trajectory, prune the tree and set
        # the new root.
        rrt.prune_tree_and_set_new_root(trajectory)
        rrt.redraw_path_to_goal()
        plt.pause(VIS_PAUSE_LENGTH)
        # Move the agent along the trajectory, continuing to expand the
        # RRT as we do so.
        rrt.move_along_path_until_done(
            trajectory[::-1], keep_stepping=True, visualize=False
        )
    plt.show()


if __name__ == "__main__":
    run_rrt_star()
