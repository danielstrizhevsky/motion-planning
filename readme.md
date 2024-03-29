# Motion Planning algorithms
Implementations and interactive visualizations of motion planning algorithms. Currently, the following algorithms are implemented:
- RRT
    - Rapidly Exploring Random Tree.
- RRT*
    - Probabilistically Optimal RRT that rewires the tree to try to improve upon existing paths.
- Anytime RRT*
    - An implementation of RRT* that allows for planning while the agent moves. Once a path to the goal is found, the agent begins following a set trajectory, but the tree continues growing from the end of this trajectory. This allows for continuous improvement of the path to the goal while the agent is moving.

See them in action below!

---
## Anytime RRT*

<p float="left">
    <img src="gifs/anytime_rrt_star.gif" width="300" height="300"/>
    <img src="gifs/anytime_rrt_star2.gif" width="300" height="300"/>
</p>

---
## RRT*

<p float="left">
    <img src="gifs/rrt_star.gif" width="300" height="300"/>
    <img src="gifs/rrt_star2.gif" width="300" height="300"/>
</p>

---
## RRT

<img src="gifs/rrt.gif" width="300" height="300"/>

---
## Requirements
- `matplotlib`

---
## Usage
In the top-level directory, run
```
export PYTHONPATH="."
```
Then, run any of the following:
```
python motion_planning/rrt/rrt.py
python motion_planning/rrt/rrt_star.py
python motion_planning/rrt/anytime_rrt_star.py
```
If running the anytime RRT* algorithm, you can click anywhere on the visualization to move the goal there! For now, `ctrl+c` to exit.

You can also add/remove/edit obstacles, change the start or goal position, and update any parameters in `config/rrt_config.py`.

---
## Resources
- Sampling-based Algorithms for Optimal Motion Planning (https://arxiv.org/abs/1105.1186)
- Anytime Motion Planning using the RRT* (https://dspace.mit.edu/handle/1721.1/63170)