#!/usr/bin/env python3
import wpilib
from ctre import WPI_VictorSPX, ControlMode

class Transit:

    def __init__(self, transit_motor_id, speed=0.75):
        self.transit_motor = WPI_VictorSPX(transit_motor_id)

        self.transit_speed = 0
        self.max_speed = 0.75

    def forward(self):
        """Move balls away from the intake"""
        self.transit_speed = -1 * self.max_speed

    def backward(self):
        """Move balls towards the intake"""
        self.transit_speed = 1 * self.max_speed
    
    def stop(self):
        """Stop the transit"""
        self.transit_speed = 0

    def set_max_speed(self, max_speed):
        """Update max transit speed"""
        self.max_speed = max_speed

    def update(self):
        """Update the transit"""
        self.transit_motor.set(ControlMode.PercentOutput, self.transit_speed)
