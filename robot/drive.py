#!/usr/bin/env python3
import wpilib
from ctre import WPI_TalonSRX, ControlMode

class Drive:

    def __init__(self, rightMaster, leftMaster, rightAlt, leftAlt):
        self.rightMaster = rightMaster
        self.leftMaster = leftMaster
        self.rightAlt = rightAlt
        self.leftAlt = leftAlt


    def arcadeDrive(self, xAxis, tAxis, controlMode: ControlMode):
        self.leftMaster.set(controlMode, xAxis + tAxis)
        self.rightMaster.set(controlMode, xAxis - tAxis)

    def tankDrive(self, leftAxis, rightAxis, controlMode: ControlMode):
        self.leftMaster.set(controlMode.PercentOutput, leftAxis)
        self.rightMaster.set(ControlMode.PercentOutput, rightAxis)
