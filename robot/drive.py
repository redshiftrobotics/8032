#!/usr/bin/env python3
import wpilib
from ctre import WPI_TalonSRX, ControlMode

class Drive:

    def __init__(self, rightMaster, leftMaster, rightAlt, leftAlt):
        self.rightMaster = rightMaster
        self.leftMaster = leftMaster
        self.rightAlt = rightAlt
        self.leftAlt = leftAlt

        self.leftSpeed = 0
        self.rightSpeed = 0
        self.controlMode = ControlMode.PercentOutput


    def arcadeDrive(self, xAxis, tAxis, controlMode):
        self.leftSpeed = xAxis + tAxis
        self.rightSpeed = xAxis - tAxis
        self.controlMode = controlMode

    def tankDrive(self, leftAxis, rightAxis, controlMode):
        self.leftSpeed = leftAxis
        self.rightSpeed = rightAxis
        self.controlMode = controlMode
    
    def update(self):
        self.leftMaster.set(self.controlMode, self.leftSpeed)
        self.rightMaster.set(self.controlMode, self.rightSpeed)
    
    def stop(self):
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.controlMode = ControlMode.PercentOutput
