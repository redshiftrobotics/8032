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
        self.intake_enabled = False
        
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
    
    def enable_collect(self):
        """Runs intake"""
        self.intake_enabled = True
    
    def disable_collect(self):
        """Stops intake"""
        self.intake_enabled = False
    
    def toggle_collect(self):
        """Toggles whether the intake is running"""
        self.intake_enabled = not self.intake_enabled
        
    def stop(self):
        """Stops the intake"""
        self.collect_speed = 0
    
    def get_intake_enabled(self):
        """Returns whether the intake is enabled"""
        return self.intake_enabled

    def update(self):
        """Updates the values if they are changed"""
        if self.piston.get() != self.piston_target:
            self.piston.set(self.piston_target)

        if self.intake_enabled:
            self.intake_motor.set(ControlMode.PercentOutput, self.collect_speed)
        else:
            self.intake_motor.set(ControlMode.PercentOutput, 0)

        