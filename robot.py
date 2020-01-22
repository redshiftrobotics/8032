#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer


class Robot(wpilib.TimedRobot):
    def threshhold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)

    def robotInit(self):
		# Sets the speed
        self.speed = 0.5
        
        self.yAxis = 0
        self.tAxis = 0

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # The [a] button debounced
        self.button = ButtonDebouncer(wpilib.Joystick(0), 0)
        
        # Motors for driving
        self.frontLeftMotor = wpilib.Talon(2)
        self.rearLeftMotor = wpilib.Talon(0)
        self.frontRightMotor = wpilib.Talon(3)
        self.rearRightMotor = wpilib.Talon(1)

        # Motor controller groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # Robot drivetrain
        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()
        
    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        
        # Drive forward for two seconds.
        if self.timer.get() < 2:
          self.myRobot.tankDrive(1, 1)

    def teleopInit(self):
        self.myRobot.setSafetyEnabled(True)

        # Tests setting the debounce period
        self.button.set_debounce_period(0.8)

    def teleopPeriodic(self):
        self.yAxis = self.threshhold(self.joystick.getRawAxis(2), 0.5)
        self.tAxis = self.threshhold(-self.joystick.getRawAxis(1), 0.05)
        
        # Drives with arcade steering
        self.myRobot.arcadeDrive(self.yAxis * self.speed, self.tAxis * self.speed)
        
        # Debug joysticks
        self.logger.info("X1: {} Y1: {} X2: {} Y2: {}".format(
            self.joystick.getX(), 
            self.joystick.getY(), 
            self.joystick.getAxis(4), 
            self.joystick.getThrottle()
        ))

if __name__ == "__main__":
    wpilib.run(Robot)
