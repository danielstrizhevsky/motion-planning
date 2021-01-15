# Motion Planning algorithms
Implementations and interactive visualizations of motion planning algorithms. Currently, the following algorithms are implemented:
- RRT
- RRT*
- Anytime RRT*

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
Run any of the following:
```
python rrt.py
python rrt_star.py
python anytime_rrt_star.py
```
If running the anytime RRT* algorithm, you can click anywhere on the visualization to move the goal there! For now, `ctrl+c` to exit.

You can also add/remove/edit obstacles, change the start or goal position, and update any parameters in `config/rrt_config.py`.

---
## Resources
- Sampling-based Algorithms for Optimal Motion Planning (https://arxiv.org/abs/1105.1186)
- Anytime Motion Planning using the RRT* (https://dspace.mit.edu/handle/1721.1/63170)