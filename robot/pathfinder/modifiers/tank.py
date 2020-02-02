import math
from copy import copy

def pathfinder_modify_tank(original, wheelbase_width):
    length = len(original)
    left_traj = []
    right_traj = []
    w = wheelbase_width / 2
    
    for i in range(length):
        seg = original[i]
        left = copy(seg)
        right = copy(seg)
        
        cos_angle = math.cos(seg.heading)
        sin_angle = math.sin(seg.heading)
        
        left.x = seg.x - (w * sin_angle)
        left.y = seg.y + (w * cos_angle)
        
        if (i > 0):
            last = left_traj[i - 1]
            distance = math.sqrt(
                (left.x - last.x) * (left.x - last.x)
                + (left.y - last.y) * (left.y - last.y)
            )
            
            left.position = last.position + distance
            left.velocity = distance / seg.dt
            left.acceleration = (left.velocity - last.velocity) / seg.dt
            left.jerk = (left.acceleration - last.acceleration) / seg.dt
        
        
        right.x = seg.x + (w * sin_angle)
        right.y = seg.y - (w * cos_angle)
        
        if (i > 0):
            last = right_traj[i - 1]
            distance = math.sqrt(
                (right.x - last.x) * (right.x - last.x)
                + (right.y - last.y) * (right.y - last.y)
            )
            
            right.position = last.position + distance
            right.velocity = distance / seg.dt
            right.acceleration = (right.velocity - last.velocity) / seg.dt
            right.jerk = (right.acceleration - last.acceleration) / seg.dt
        
        
        left_traj.append(left)
        right_traj.append(right)
    return left_traj, right_traj
