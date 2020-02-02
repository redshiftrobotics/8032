import sys
sys.path.insert(0, "pathfinder/")

import generator
import modifiers
import followers
from utils import Waypoint, Segment, print_object, SWERVE_DEFAULT
from mathutil import r2d, d2r, bound_angle

sys.path.insert(0, "pathfinder/fit/")
import hermite

SAMPLES_FAST = 1000
SAMPLES_LOW = SAMPLES_FAST*10
SAMPLES_HIGH = SAMPLES_LOW*10
