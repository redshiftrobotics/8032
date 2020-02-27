#!/usr/bin/env python3
import wpilib
from ctre import WPI_VictorSPX, ControlMode

class Transit:

    def __init__(self, transit_motor_id, inverted=1):
        self.transit_motor = WPI_VictorSPX(transit_motor_id)

        self.transit_speed = 0
        self.inverted = inverted
        self.transit_speed_mult = 0.6

    def forward(self):
        """Move balls away from the intake"""
        self.transit_speed = self.inverted * self.transit_speed_mult

    def backward(self):
        """Move balls towards the intake"""
        self.transit_speed = -1 * self.inverted * self.transit_speed_mult

    def stop(self):
        """Stop the transit"""
        self.transit_speed = 0

    def speed(self, speed):
        """Update max transit speed"""
        self.transit_speed = speed * self.transit_speed_mult

    def update(self):
        """Update the transit"""
        self.transit_motor.set(ControlMode.PercentOutput, self.transit_speed)