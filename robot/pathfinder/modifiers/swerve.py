from copy import copy
import sys

sys.path.append("../")
from utils import SWERVE_DEFAULT

def pf_modify_swerve_default(original, wheelbase_width, wheelbase_depth):
    length = len(original)
    front_left = []
    front_right = []
    back_left = []
    back_right = []

    for i in range(length):
        seg = original[i]
        fl = copy(seg)
        fr = copy(seg)
        bl = copy(seg)
        br = copy(seg)
        
        fl.x = seg.x - wheelbase_width / 2
        fl.y = seg.y + wheelbase_depth / 2
        fr.x = seg.x + wheelbase_width / 2
        fr.y = seg.y + wheelbase_depth / 2
        
        bl.x = seg.x - wheelbase_width / 2
        bl.y = seg.y - wheelbase_depth / 2
        br.x = seg.x + wheelbase_width / 2
        br.y = seg.y - wheelbase_depth / 2
        
        front_left.append(fl)
        front_right.append(fr)
        back_left.append(bl)
        back_right.append(br)
    return front_left, front_right, back_left, back_right

def pathfinder_modify_swerve(original, wheelbase_width, wheelbase_depth, mode):
    if (mode == SWERVE_DEFAULT):
        return pf_modify_swerve_default(original, wheelbase_width, wheelbase_depth)
    

