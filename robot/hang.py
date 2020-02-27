#!/usr/bin/env python3
import wpilib
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

class Hang:

    def __init__(self, left_motor_id, right_motor_id, max_speed, stop_point, kP=0.05, kD=0, kI=0):
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        self.kP = kP
        self.kD = kD
        self.kI = kI

        self.master_motor = WPI_TalonSRX(left_motor_id)
        self.master_motor.setInverted(False)
        self.master_motor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )
        
        self.master_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.master_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.master_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

        self.follower_motor = WPI_TalonSRX(right_motor_id)
        self.follower_motor.setInverted(True)
        #self.follower_motor.follow(self.master_motor)

        self.hang_tgt = 0
        self.hang_mode = ControlMode.PercentOutput
        self.max_speed = max_speed
        self.stop_point = stop_point
    
    def set_pid(self, kP, kD, kI):
        self.kP = kP
        self.kD = kD
        self.kI = kI

        self.master_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.master_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.master_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

    def extend(self):
        """Extends hang"""
        self.hang_tgt = self.stop_point
        self.hang_mode = ControlMode.PercentOutput#ControlMode.Position

    def move(self, speed_mult = 1):
        """Moves hang"""
        self.hang_tgt = self.max_speed * speed_mult
        self.hang_mode = ControlMode.PercentOutput

    def stop(self):
        """Stops hang"""
        self.hang_tgt = 0

    def update(self):
        """Updates the values if they are changed"""
        self.follower_motor.set(self.hang_mode, self.hang_tgt)
        self.master_motor.set(self.hang_mode, self.hang_tgt)
