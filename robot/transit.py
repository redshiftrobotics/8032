#!/usr/bin/env python3
import wpilib
from ctre import WPI_VictorSPX, ControlMode, NeutralMode, FeedbackDevice

class Transit:

    def __init__(self, transit_motor_channel, speed=0.75):
        self.transit_motor = WPI_VictorSPX(transit_motor_channel)

        self.transit_speed = 0
        self.speed = 0.75

    def forward(self):
        """Extends hang"""
        self.transit_speed = 1 * self.speed

    def backward(self):
        """Retracts hang"""
        self.transit_speed = -1 * self.speed
    
    def stop(self):
        self.transit_speed = 0

    def update(self):
        """Updates the values if they are changed"""
        self.transit_motor.set(self.transit_speed)
