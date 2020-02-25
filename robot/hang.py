#!/usr/bin/env python3
import wpilib
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

class Hang:

    def __init__(self, left_motor_id, right_motor_id, extend_speed, retract_speed, stop_point):
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        self.master_motor = WPI_TalonSRX(left_motor_id)
        self.master_motor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.follower_motor = WPI_TalonSRX(right_motor_id)
        self.follower_motor.follow(self.master_motor)
        self.follower_motor.setInverted(True)


        self.hang_speed = 0

        self.extend_speed = extend_speed
        self.retract_speed = retract_speed

    def extend(self):
        """Extends hang"""
        self.hang_speed = self.extend_speed

    def retract(self, speed_mult = 1):
        """Retracts hang"""
        self.hang_speed = self.retract_speed * speed_mult

    def stop(self):
        """Stops hang"""
        self.hang_speed = 0

    def update(self):
        """Updates the values if they are changed"""
        self.master_motor.set(ControlMode.PercentOutput, self.hang_speed)
        print(self.hang_speed)

        