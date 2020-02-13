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
        self.last_error = 0
        self.segment = 0
        self.start_time = 0
        self.tlen = len(trajectory)

    def setTrajectory(self, trajectory):
        self.trajectory = trajectory
        self.tlen = len(trajectory)
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
        self.start_time = 0

    def start(self, start_time):
        self.reset()
        self.start_time = start_time

    def calculate(self, encoder_tick, now):
        if self.segment >= self.tlen:
            self.finished = 1
            self.output = 0
            self.heading = self.trajectory[-1].heading
            return 0.0
        else:
            out = pathfinder_follow_encoder2(self.cfg, self, self.trajectory[self.segment], self.tlen, encoder_tick)

            time_passed = now-self.start_time
            print(time_passed, now, self.trajectory[self.segment].t, self.segment)

            while self.trajectory[self.segment].t < time_passed:
                self.segment = self.segment + 1

                if self.segment >= self.tlen:
                    break

            return out

    def getHeading(self):
        return self.heading

    def getSegment(self):
        return self.trajectory[self.segment]

    def isFinished(self):
        return self.segment >= len(self.trajectory)

def pathfinder_follow_encoder2(c, follower, s, trajectory_length, encoder_tick):
    distance_covered = (float(encoder_tick) - float(c.initial_position)) /  float(c.ticks_per_revolution)
    distance_covered *= c.wheel_circumference
    
    if (follower.segment < trajectory_length):
        follower.finished = 0
        error = s.position - distance_covered
        feedback_value = (c.kp * error) + (c.kd * ((error - follower.last_error) / s.dt))
        feedforward_value = ((c.kv * s.velocity) + (c.ka * s.acceleration))
        calculated_value = feedback_value + feedforward_value
        
        follower.last_error = error
        follower.heading = s.heading
        follower.output = calculated_value
        #follower.segment = follower.segment + 1
        return calculated_value
    else:
        follower.finished = 1
        return 0.0
   
##########################################################################################
##########################################################################################
##########################################################################################

def pathfinder_follow_distance(c, follower, trajectory, trajectory_length, distance):
    segment = follower.segment
    if (segment >= trajectory_length):
        follower.finished = 1
        follower.output = 0.0
        last = trajectory[trajectory_length - 1]
        follower.heading = last.heading
        return 0.0
    else:
        return pathfinder_follow_distance2(c, follower, trajectory[segment], trajectory_length, distance)
   


def pathfinder_follow_distance2(c, follower, s, trajectory_length, distance):
    if (follower.segment < trajectory_length):
        follower.finished = 0
        error = s.position - distance
        calculated_value = c.kp * error + c.kd * ((error - follower.last_error) / s.dt) + (c.kv * s.velocity + c.ka * s.acceleration)
        
        follower.last_error = error
        follower.heading = s.heading
        follower.output = calculated_value
        follower.segment = follower.segment + 1
        return calculated_value
    else:
        follower.finished = 1
        return 0.0
   
