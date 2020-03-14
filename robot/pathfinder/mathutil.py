import math

def bound_radians(angle):
    newAngle = angle % math.tau
    if (newAngle < 0):
        newAngle = math.tau + newAngle
    return newAngle

def bound_angle(angle):
    newAngle = angle % 360
    if (newAngle > 180):
        newAngle = newAngle-360
    return newAngle

def r2d(angleInRads):
    return angleInRads * 180 / math.pi

def d2r(angleInDegrees):
    return angleInDegrees * math.pi / 180

def signum(num):
    return math.copysign(1, num)