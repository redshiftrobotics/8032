import wpilib
import wpilib.drive
import pathfinder as pf


class TrajectoryFollower:
    WHEEL_DIAMETER = 0.5
    KV = 5.99
    KA = 0.717
    KS = 1.08

    def __init__(self):
        

