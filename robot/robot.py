#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

from intake import Intake
from transit import Transit

class Robot(wpilib.TimedRobot):
    def threshold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)

    def robotInit(self):
        self.kSlotIdx = 0
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        
		    # Sets the speed
        self.speed = 0.4
        self.ySpeed = 1
        self.tSpeed = 0.75
        self.baseIntakeSpeed = 5

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # joysticks 1 & 2 on the driver station
        self.driverJoystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()

        # TODO: Fix module number
        self.compressor = wpilib.Compressor(0)

        # TODO: Fix module numbers
        self.intake = Intake(7,61,1,0)
        
        self.pistonButtonPort = 2

        self.intakeSpeedToggle = ButtonDebouncer(self.driverJoystick, 1)

        # TODO: Fix motor port
        self.transit = Transit(6)
        self.transitForward = 3
        self.transitBackward = 4

        # Talon CAN devices
        # Setup Master motors for each side
        self.leftMaster = WPI_TalonSRX(4) # Front left Motor
        self.leftMaster.setInverted(False)
        self.leftMaster.setSensorPhase(False)
        self.leftMaster.setNeutralMode(NeutralMode.Brake)

        self.rightMaster = WPI_TalonSRX(5) # Front right Motor
        self.rightMaster.setInverted(True)
        self.rightMaster.setSensorPhase(False)
        self.rightMaster.setNeutralMode(NeutralMode.Brake)

        # Setup Follower motors for each side
        self.leftFollower0 = WPI_TalonSRX(2) # Back left motor
        self.leftFollower0.setInverted(False)
        self.leftFollower0.follow(self.leftMaster)
        self.leftFollower0.setNeutralMode(NeutralMode.Brake)

        self.rightFollower0 = WPI_TalonSRX(3) # Back right motor
        self.rightFollower0.setInverted(False)
        self.rightFollower0.follow(self.leftMaster)
        self.rightFollower0.setNeutralMode(NeutralMode.Brake)

        # Setup encoders
        self.leftMaster.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.rightMaster.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        # Setup Differential Drive
        self.drive = DifferentialDrive(self.leftMaster, self.rightMaster)
        self.drive.setDeadband(0) # Disable auto joystick thresholding

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        pass

    def teleopInit(self):
        self.compressor.start()
        pass 

    def teleopPeriodic(self):
        # Get max speed
        self.speed = (-self.driverJoystick.getRawAxis(3) + 1)/2

        if self.driverJoystick.getRawButton(self.pistonButtonPort):
            self.intake.extend()
        else:
            self.intake.retract()

        if self.intakeSpeedToggle.get():
            speedAverage = self.leftEncoder.getMotorOutputPercent() + self.rightEncoder.getMotorOutputPercent()
            speedAverage /= 2
            speedAverage = abs(speedAverage)

            self.intake.speed(speedAverage + 0.5)
        else:
            self.intake.speed(0)
        

        if self.driverJoystick.getRawButton(self.transitForward):
            self.transit.forward()
        elif self.driverJoystick.getRawButton(self.transitBackward):
            self.transit.backward()
        else:
            self.transit.stop()

        # Get turn and movement speeds
        self.tAxis = self.threshold(self.driverJoystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed
        self.yAxis = self.threshold(-self.driverJoystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed

        self.transit.update()
        self.intake.update()
        self.drive.arcadeDrive(self.yAxis, self.tAxis)
    
if __name__ == "__main__":
    wpilib.run(Robot)
