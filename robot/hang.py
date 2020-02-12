#!/usr/bin/env python3
import wpilib
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

class Hang:

    def __init__(self, left_motor_id, right_motor_id, extend_speed, retract_speed, stop_point):
        self.master_motor = WPI_TalonSRX(left_motor_id)

        self.follower_motor = WPI_TalonSRX(right_motor_id)
        self.follower_motor.follow(self.master_motor)
        self.follower_motor.setInverted(True)


        self.hang_speed = 0

        self.extend_speed = extend_speed
        self.retract_speed = retract_speed

    def extend(self):
        """Extends hang"""
        self.hang_speed = self.extend_speed

    def retract(self):
        """Retracts hang"""
        self.hang_speed = self.retract_speed

    def update(self):
        """Updates the values if they are changed"""
        self.intake_motor.set(ControlMode.Speed, self.collect_speed)

        