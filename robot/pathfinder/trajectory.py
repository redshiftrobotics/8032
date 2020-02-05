import math
import copy

from pathfinder.utils import Segment, print_object

class TrajectoryInfo:
    def __init__(self, filter1, filter2, length, dt, u, v, impulse):
        self.filter1 = filter1
        self.filter2 = filter2
        self.length = length
        self.dt = dt
        self.u = u
        self.v = v
        self.impulse = impulse

def pf_trajectory_copy(src, dest):
    for i in range(len(src)):
        dest[i] = src[i]

def pf_trajectory_prepare(c):
    max_a2 = c.max_a * c.max_a
    max_j2 = c.max_j * c.max_j
    
    checked_max_v = min(c.max_v,
        (-(max_a2) + math.sqrt(max_a2 * max_a2 + 4 * (max_j2 * c.max_a * c.dest_pos)))
        / (2 * c.max_j)
    )
    
    filter1 = int(math.ceil((checked_max_v / c.max_a) / c.dt))
    filter2 = int(math.ceil((c.max_a / c.max_j) / c.dt))
    
    impulse = (c.dest_pos / checked_max_v) / c.dt
    time =  int(math.ceil(filter1 + filter2 + impulse))
    
    info = TrajectoryInfo(filter1, filter2, time, c.dt, 0, checked_max_v, impulse)
    return info

def pf_trajectory_create(info, c, seg):
    ret = pf_trajectory_fromSecondOrderFilter(info.filter1, info.filter2, info.dt, info.u, info.v, info.impulse, info.length, seg)
    
    if (ret < 0):
        return ret
    
    d_theta = c.dest_theta - c.src_theta
    for i in range(len(seg)):
        seg[i].heading = c.src_theta + d_theta * (seg[i].position) / (seg[info.length - 1].position)
    return 0

def pf_trajectory_fromSecondOrderFilter(filter_1_l, filter_2_l, dt, u, v, impulse, length, t):
    last_section = Segment(dt, 0, 0, 0, u, 0, 0)
    
    if (length < 0):
        # Error
        return -1
    
    f1_buffer = [0]*length
    f1_buffer[0] = (u / v) * filter_1_l
    f2 = 0

    a = []
    
    for i in range(length):
        input_num = min(impulse, 1)
        if (input_num < 1):
            input_num -= 1
            impulse = 0
        else:
            impulse -= input_num

        f1_last = 0
        
        if (i > 0):
            f1_last = f1_buffer[i - 1]
        else:
            f1_last = f1_buffer[0]

        f1_buffer[i] = max(0.0, min(filter_1_l, f1_last + input_num))

        f2 = 0
        for j in range(filter_2_l):
            if (i - j < 0):
                break
            f2 += f1_buffer[i - j]
        
        f2 = f2 / filter_1_l

        t[i].velocity = f2 / filter_2_l * v

        t[i].position = (last_section.velocity + t[i].velocity) / 2.0 * dt + last_section.position

        t[i].x = t[i].position
        t[i].y = 0

        t[i].acceleration = (t[i].velocity - last_section.velocity) / dt
        t[i].jerk = (t[i].acceleration - last_section.acceleration) / dt
        t[i].dt = dt

        last_section = t[i]

        t[i] = copy.copy(t[i])
    return 0