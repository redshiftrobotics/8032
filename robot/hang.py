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

        self.left_motor = WPI_TalonSRX(left_motor_id)
        self.left_motor.setInverted(False)
        self.left_motor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )
        
        self.left_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.left_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.left_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

        self.right_motor = WPI_TalonSRX(right_motor_id)
        self.right_motor.setInverted(True)
        self.right_motor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.right_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.right_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.right_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

        self.right_tgt = 0
        self.left_tgt = 0
        self.hang_mode = ControlMode.PercentOutput
        self.max_speed = max_speed
        self.stop_point = stop_point
    
    def set_pid(self, kP, kD, kI):
        self.kP = kP
        self.kD = kD
        self.kI = kI

        self.left_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.left_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.left_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

        self.right_motor.config_kP(self.kPIDLoopIdx, self.kP, self.kTimeoutMs)
        self.right_motor.config_kD(self.kPIDLoopIdx, self.kD, self.kTimeoutMs)
        self.right_motor.config_kI(self.kPIDLoopIdx, self.kI, self.kTimeoutMs)

    def retract(self):
        """Retracts hang"""
        self.right_tgt = -self.max_speed
        self.left_tgt = -self.max_speed
        self.hang_mode = ControlMode.PercentOutput

    def move(self, left, right):
        """Moves hang"""
        self.right_tgt = self.max_speed * right
        self.left_tgt = self.max_speed * left
        self.hang_mode = ControlMode.PercentOutput

    def stop(self):
        """Stops hang"""
        self.right_tgt = 0
        self.left_tgt = 0

    def update(self):
        """Updates the values if they are changed"""
        self.right_motor.set(self.hang_mode, self.right_tgt)
        self.left_motor.set(self.hang_mode, self.left_tgt)
