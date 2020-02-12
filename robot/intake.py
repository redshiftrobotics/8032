#!/usr/bin/env python3
import wpilib
from ctre import VictorSPX, ControlMode

class Intake:

    def __init__(self, intake_motor_channel, piston_module_number, piston_forward, piston_reverse):
        self.intake_motor = VictorSPX(intake_motor_channel)
        self.piston = wpilib.DoubleSolenoid(piston_module_number, piston_forward, piston_reverse)

        self.piston_target = self.piston.Value.kReverse

        self.collect_speed = 0
        
    def extend(self):
        """Extends pistons"""
        self.piston_target = self.piston.Value.kForward

    def retract(self):
        """Retracts pistons"""
        self.piston_target = self.piston.Value.kReverse

    def speed(self, speed):
        """Sets the intake speed"""
        self.collect_speed = speed

    def update(self):
        """Updates the values if they are changed"""
        if self.piston.get() != self.piston_target:
            self.piston.set(self.piston_target)

        self.intake_motor.set(ControlMode.PercentOutput, self.collect_speed)

        