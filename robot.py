#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    tank drive.
"""

import wpilib
from wpilib.drive import DifferentialDrive

from networktables import NetworkTables

#wassup dood reeee
class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
		# Sets the speed
        self.speed = 0.5

        self.sd = NetworkTables.getTable('SmartDashboard')

        # object that handles basic drive operations for the robot
        self.frontLeftMotor = wpilib.Talon(2)
        self.rearLeftMotor = wpilib.Talon(0)
        self.frontRightMotor = wpilib.Talon(3)
        self.rearRightMotor = wpilib.Talon(1)

        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(
            self.frontRightMotor, self.rearRightMotor
        )

        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.myRobot.tankDrive(
			self.joystick.getRawAxis(1) * -1 * (self.speed + self.joystick.getRawAxis(5)/4 + self.joystick.getRawAxis(4)), 
			self.joystick.getRawAxis(3) * (self.speed + self.joystick.getRawAxis(5)/4 + self.joystick.getRawAxis(4))
			)

        self.logger.info("Joystick value: %d", self.joystick.getRawAxis(1))

        self.sd.putNumber('test', 3)


if __name__ == "__main__":
    wpilib.run(MyRobot)
