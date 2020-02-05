import math

from pathfinder.spline import pf_spline_distance, pf_spline_progress_for_distance, pf_spline_coords, pf_spline_angle
from pathfinder.trajectory import pf_trajectory_prepare, pf_trajectory_create
from pathfinder.utils import Segment

class TrajectoryConfig:
    def __init__(self, dt, max_v, max_a, max_j, src_v, src_theta, dest_pos, dest_v, dest_theta, sample_count):
        self.dt = dt
        self.max_v = max_v
        self.max_a = max_a
        self.max_j = max_j
        self.src_v = src_v
        self.src_theta = src_theta
        self.dest_pos = dest_pos
        self.dest_v = dest_v
        self.dest_theta = dest_theta
        self.sample_count = sample_count

class Spline:
    def __init__(self, a=0, b=0, c=0, d=0, e=0, x_offset=0, y_offset=0, angle_offset=0, knot_distance=0, arc_length=0):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.angle_offset = angle_offset
        self.knot_distance = knot_distance
        self.arc_length = arc_length

class TrajectoryCandidate:
    def __init__(self, saptr=[], laptr=[], totalLength=0, length=0, path_length=0, info=None, config=None):
        self.saptr = saptr
        self.laptr = laptr
        self.totalLength = totalLength
        self.length = length
        self.path_length = path_length
        self.info = info
        self.config = config

def pathfinder_prepare(path, fit, sample_count, dt, max_velocity, max_acceleration, max_jerk):
    path_length = len(path)
    if (path_length < 2):
        return -1
    
    cand = TrajectoryCandidate()

    totalLength = 0
    
    for i in range(path_length-1):
        s = Spline()
        fit(path[i], path[i+1], s)
        dist = pf_spline_distance(s, sample_count)
        cand.saptr.append(s)
        cand.laptr.append(dist)
        totalLength += dist
    
    
    config = TrajectoryConfig(dt, max_velocity, max_acceleration, max_jerk, 0, path[0].angle,
        totalLength, 0, path[0].angle, sample_count)
    info = pf_trajectory_prepare(config)
    trajectory_length = info.length

    cand.totalLength = totalLength
    cand.length = trajectory_length
    cand.path_length = path_length
    cand.info = info
    cand.config = config
    
    return cand

def pathfinder_generate(c):
    trajectory_length = c.length
    path_length = c.path_length
    totalLength = c.totalLength

    segments = [Segment()]*trajectory_length
    
    splines = c.saptr
    splineLengths = c.laptr
    
    trajectory_status = pf_trajectory_create(c.info, c.config, segments)
    if (trajectory_status < 0):
        return None
    
    spline_i = 0
    spline_pos_initial = 0
    splines_complete = 0
    
    for i in range(trajectory_length):
        pos = segments[i].position

        found = 0
        while (not found):
            pos_relative = pos - spline_pos_initial
            if (pos_relative <= splineLengths[spline_i]):
                si = splines[spline_i]
                percentage = pf_spline_progress_for_distance(si, pos_relative, c.config.sample_count)
                coords = pf_spline_coords(si, percentage)
                segments[i].heading = pf_spline_angle(si, percentage)
                segments[i].x = coords.x
                segments[i].y = coords.y
                found = 1
            elif (spline_i < path_length - 2):
                splines_complete += splineLengths[spline_i]
                spline_pos_initial = splines_complete
                spline_i += 1
            else:
                si = splines[path_length - 2]
                segments[i].heading = pf_spline_angle(si, 1.0)
                coords = pf_spline_coords(si, 1.0)
                segments[i].x = coords.x
                segments[i].y = coords.y
                found = 1
    return segments
def generate_trajectory(path, fit, sample_count, dt, max_velocity, max_acceleration, max_jerk):
    cand = pathfinder_prepare(path, fit, sample_count, dt, max_velocity, max_acceleration, max_jerk)
    return pathfinder_generate(cand)