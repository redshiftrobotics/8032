import math

class EncoderConfig:
    def __init__(self, initial_position=0, ticks_per_revolution=0, wheel_circumference=0, kp=0, ki=0, kd=0, kv=0, ka=0):
        self.initial_position = initial_position
        self.ticks_per_revolution = ticks_per_revolution
        self.wheel_circumference = wheel_circumference
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kv = kv
        self.ka = ka

## Modified from: https://github.com/robotpy/robotpy-pathfinder/blob/master/pathfinder/followers.py
class EncoderFollower:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.cfg = EncoderConfig()

    def setTrajectory(self, trajectory):
        self.trajectory = trajectory
        self.reset()

    def configurePIDVA(self, kp, ki, kd, kv, ka):
        self.cfg.kp = kp
        self.cfg.ki = ki
        self.cfg.kd = kd
        self.cfg.kv = kv
        self.cfg.ka = ka

    def configureEncoder(self, initial_position, ticks_per_revolution, wheel_diameter):
        self.cfg.initial_position = initial_position
        self.cfg.ticks_per_revolution = ticks_per_revolution
        self.cfg.wheel_circumference = math.pi * wheel_diameter

    def reset(self):
        self.last_error = 0
        self.segment = 0

    def calculate(self, encoder_tick):
        tlen = len(self.trajectory)
        if self.segment >= tlen:
            self.finished = 1
            self.output = 0
            self.heading = self.trajectory[-1].heading
            return 0.0
        else:
            return pathfinder_follow_encoder2(self.cfg, self, self.trajectory[self.segment], tlen, encoder_tick)

    def getHeading(self):
        return self.heading

    def getSegment(self):
        return self.trajectory[self.segment]

    def isFinished(self):
        return self.segment >= len(self.trajectory)

def pathfinder_follow_encoder2(c, follower, s, trajectory_length, encoder_tick):
    distance_covered = (float(encoder_tick) - float(c.initial_position)) /  float(c.ticks_per_revolution)
    distance_covered = distance_covered * c.wheel_circumference
    
    if (follower.segment < trajectory_length):
        follower.finished = 0
        error = s.position - distance_covered
        calculated_value = c.kp * error + c.kd * ((error - follower.last_error) / s.dt) + (c.kv * s.velocity + c.ka * s.acceleration)
        
        follower.last_error = error
        follower.heading = s.heading
        follower.output = calculated_value
        follower.segment = follower.segment + 1
        return calculated_value
    else:
        follower.finished = 1
        return 0.0
   
