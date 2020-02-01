#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from state import State
from state import DRIVE_FORWARD_TWO_SEC, FREEZE, TANK_DRIVE_NORMAL

class Robot(wpilib.TimedRobot):
    def threshold(self, value, limit):
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
        
        self._state = State(self, FREEZE)
        
    @property
    def state(self):
        return self._state.state
        
    @state.setter
    def state(self, new_state):
        self._state.dispatch(new_state)
        
    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        self._state.update()

        if self.state == DRIVE_FORWARD_TWO_SEC:
            self.myRobot.tankDrive(0.3, 0.3)
        elif self.state == FREEZE:
            self.myRobot.tankDrive(0, 0)

    def teleopInit(self):
        self.myRobot.setSafetyEnabled(True)

        # Tests setting the debounce period
        self.button.set_debounce_period(0.8)

    def teleopPeriodic(self):      
        self._state.update()

        self.yAxis = self.threshold(self.joystick.getRawAxis(2), 0.5)
        self.tAxis = self.threshold(-self.joystick.getRawAxis(1), 0.05)

        # Debug joysticks
        self.logger.info("X1: {} Y1: {} X2: {} Y2: {}".format(
            self.joystick.getX(), 
            self.joystick.getY(), 
            self.joystick.getAxis(4), 
            self.joystick.getThrottle()
        ))

        if self.state == TANK_DRIVE_NORMAL:
            # Drives with arcade steering
            self.myRobot.arcadeDrive(self.yAxis * self.speed,
                                     self.tAxis * self.speed)
        elif self.state == FREEZE:
            self.myRobot.tankDrive(0, 0)

if __name__ == "__main__":
    wpilib.run(Robot)
