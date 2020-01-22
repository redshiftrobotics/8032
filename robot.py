#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from colorSensorV3 import ColorSensorV3
from limelight import LimeLight
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from state import State
from state import DRIVE_FORWARD_TWO_SEC, FREEZE, TANK_DRIVE_NORMAL


class Robot(wpilib.TimedRobot):

    def robotInit(self):
		# Sets the speed
        self.speed = 0.5

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # LimeLight
        self.limelight = LimeLight()

        # REV Color Sensor V3
        self.colorSensor = ColorSensorV3(0)

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

        # Debug joysticks
        self.logger.info("X1: {} Y1: {} X2: {} Y2: {}".format(
            self.joystick.getX(), 
            self.joystick.getY(), 
            self.joystick.getAxis(4), 
            self.joystick.getThrottle()
        ))

        # Tests the button debouncer
        self.sd.putNumber(self.button.get())
        
        # Tests ColorSensor output
        self.sd.putNumber(self.colorSensor.getGreen())

        # Tests the LimeLight's latency
        self.sd.putNumber(self.limelight.getLatency())
        
        if self.state == TANK_DRIVE_NORMAL:
            # TODO: clean up the next line; it needs to be factored out.
            self.myRobot.tankDrive(
                self.joystick.getRawAxis(1) * -1 * (self.speed + self.joystick.getRawAxis(5)/4 + self.joystick.getRawAxis(4)),
                self.joystick.getRawAxis(3) * (self.speed + self.joystick.getRawAxis(5)/4 + self.joystick.getRawAxis(4))
            )
        elif self.state == FREEZE:
            self.myRobot.tankDrive(0, 0)

if __name__ == "__main__":
    wpilib.run(Robot)
