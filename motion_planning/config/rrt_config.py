# Distance to move towards random points. This is effectively
# the max edge length.
DELTA = 5

# Constant multiplier for radius for nearby neighbors search
# for RRT*
GAMMA_RRT_STAR = DELTA * 50

# List of rectangular obstacles, ((bottom left corner), length, height))
OBSTACLES = [
    ((0, 20), 80, 20),
    ((20, 60), 80, 20),
]

# Bounds for search space
RIGHT_BOUND = 100
BOTTOM_BOUND = 100

# How fast the agent moves along a path
AGENT_SPEED = 0.1

# For rounding issues
EPSILON = 0.000000001

# How long to pause when animating
VIS_PAUSE_LENGTH = 0.0000000000001

# For anytime RRT*, how long each saved trajectory should be.
# i.e. how many edges away to set the new root
ANYTIME_TRAJECTORY_LENGTH = 5

# Starting position of the agent
AGENT_START_POSITION = (10, 10)

# Starting position of the goal ((bottom left), (top right))
GOAL_POSITION = ((85, 85), (95, 95))
