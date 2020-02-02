class Waypoint:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class Segment:
    def __init__(self, dt=0, x=0, y=0, position=0, velocity=0, acceleration=0, jerk=0, heading=0):
        self.dt = dt
        self.x = x
        self.y = y
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.jerk = jerk
        self.heading = heading

def print_object(obj):
    attrs = vars(obj)
    print(', '.join("%s: %s" % item for item in attrs.items()))

SWERVE_DEFAULT = 0