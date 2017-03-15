from types import Twist
from math import pi

M_TO_PIXELS = 200

MAX_LINEAR_SPEED = 0.5 * M_TO_PIXELS
MAX_ANGULAR_SPEED = 1.0 * 2 * pi

# The types of objects as powers of two, used for filtering shapes
WALL_MASK = 1
ROBOT_MASK = 2
RED_PUCK_MASK = 4
GREEN_PUCK_MASK = 8
BLUE_PUCK_MASK = 16
ANY_PUCK_MASK = RED_PUCK_MASK | GREEN_PUCK_MASK | BLUE_PUCK_MASK
