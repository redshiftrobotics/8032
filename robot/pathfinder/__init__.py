import pathfinder.generator
import pathfinder.hermite
import pathfinder.modifiers
import pathfinder.followers
from pathfinder.utils import Waypoint, Segment, return_object, SWERVE_DEFAULT
from pathfinder.mathutil import d2r, r2d, bound_angle, bound_radians

SAMPLES_FAST = 1000
SAMPLES_LOW = SAMPLES_FAST*10
SAMPLES_HIGH = SAMPLES_LOW*10
