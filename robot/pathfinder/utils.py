import json
from pathfinder.mathutil import bound_radians
from pathlib import Path

class Waypoint:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class Segment:
    def __init__(self, dt=0, x=0, y=0, position=0, velocity=0, acceleration=0, jerk=0, heading=0, t=0):
        self.dt = dt
        self.t = t
        self.x = x
        self.y = y
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.jerk = jerk
        self.heading = heading

def return_object(obj):
    attrs = vars(obj)
    return ', '.join("%s: %s" % item for item in attrs.items())

def read_from_pathweaver(name, filepath):
    path = Path(filepath).parent / "paths" / "output" / (name + ".wpilib.json")

    trajectory_json = {}
    with open(path) as json_data:
        trajectory_json = json.load(json_data)

    trajectory = []

    dt = 0.05
    offset_x = 0 # trajectory_json[0]["pose"]["translation"]["x"]
    offset_y = 0 # trajectory_json[0]["pose"]["translation"]["y"]

    last_segment = Segment(dt, 0, 0, 0, 0, 0, 0, 0)
    last_time = -0.01
    for seg in trajectory_json:
        gen_seg = Segment(seg["time"]-last_time,
                          seg["pose"]["translation"]["x"]-offset_x,
                          seg["pose"]["translation"]["y"]-offset_y,
                          (last_segment.velocity + seg["velocity"]) / 2.0 * dt + last_segment.position,
                          seg["velocity"],
                          seg["acceleration"],
                          (seg["acceleration"]-last_segment.acceleration)/dt,
                          bound_radians(seg["pose"]["rotation"]["radians"]),
                          seg["time"])

        trajectory.append(gen_seg)

        last_segment = gen_seg
        last_time = seg["time"]
    return trajectory

SWERVE_DEFAULT = 0
