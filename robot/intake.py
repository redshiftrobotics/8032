#!/usr/bin/env python3
import wpilib

class Intake:

    def __init__(self, intake_motor_channel, right_piston_module_number, right_piston_forward, right_piston_reverse,
                    left_piston_module_number, left_piston_forward, left_piston_reverse):
        self.intake_motor = wpilib.Talon(intake_motor_channel)
        self.right_piston = wpilib.DoubleSolenoid(right_piston_module_number, right_piston_forward, right_piston_reverse)
        self.left_piston = wpilib.DoubleSolenoid(left_piston_module_number, left_piston_forward, left_piston_reverse)

        self.right_piston_target = self.right_piston.Value.kReverse
        self.left_piston_target = self.left_piston.Value.kReverse

        self.collect_speed = 0

    def test(self, out: bool, right: bool):
        """Tests each piston"""
        if out:
            if right:
                self.right_piston.set(self.right_piston.Value.kForward)
            else:
                self.left_piston.set(self.left_piston.Value.kForward)
        else:
            if right:
                self.right_piston.set(self.right_piston.Value.kReverse)
            else:
                self.left_piston.set(self.left_piston.Value.kReverse)
        
    def extend(self):
        """Extends pistons"""
        self.right_piston_target = self.right_piston.Value.kForward
        self.left_piston_target = self.left_piston.Value.kForward

    def retract(self):
        """Retracts pistons"""
        self.right_piston_target = self.right_piston.Value.kReverse
        self.left_piston_target = self.left_piston.Value.kReverse

    def speed(self, speed):
        """Sets the intake speed"""
        self.collect_speed = speed

    def update(self):
        """Updates the values if they are changed"""
        if self.right_piston.get() != self.right_piston_target:
            self.right_piston.set(self.right_piston_target)
        if self.right_piston.get() != self.right_piston_target:
            self.right_piston.set(self.right_piston_target)

        if self.intake_motor.get() != self.collect_speed:
            self.intake_motor.set(self.collect_speed)

        