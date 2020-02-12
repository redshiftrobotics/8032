#!/usr/bin/env python3
import wpilib
from ctre import WPI_VictorSPX, ControlMode

EXTENDED = "extended"
RETRACTED = "retracted"

class Intake:

    def __init__(self, intake_motor_id, piston_module_id, piston_forward_channel, piston_reverse_channel):
        self.intake_motor = WPI_VictorSPX(intake_motor_id)
        self.collect_speed = 0

        self.piston = wpilib.DoubleSolenoid(piston_module_id, piston_forward_channel, piston_reverse_channel)
        self.piston_target = self.piston.Value.kReverse

        self.state = EXTENDED
        
    def extend(self):
        """Extends pistons"""
        self.piston_target = self.piston.Value.kForward
        self.state = EXTENDED

    def retract(self):
        """Retracts pistons"""
        self.piston_target = self.piston.Value.kReverse
        self.state = RETRACTED
    
    def toggle(self):
        """Toggles the state of the intake"""
        if self.state == EXTENDED:
            self.retract()
        else:
            self.extend()

    def speed(self, speed):
        """Sets the intake speed"""
        self.collect_speed = speed

    def update(self):
        """Updates the values if they are changed"""
        if self.piston.get() != self.piston_target:
            self.piston.set(self.piston_target)

        self.intake_motor.set(ControlMode.PercentOutput, self.collect_speed)

        