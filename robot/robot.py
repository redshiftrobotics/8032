#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice
from joystick_drive import joystick_drive

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
        
        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')
        
        # joysticks 1 on the driver station
        self.button = ButtonDebouncer(wpilib.Joystick(0), 0)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        self.joystick1 = wpilib.Joystick(1)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()
        
        # Talon CAN devices
        self.frontLeftTalon = WPI_TalonSRX(4)
        self.rearLeftTalon = WPI_TalonSRX(2)
        self.frontRightTalon = WPI_TalonSRX(5)
        self.rearRightTalon = WPI_TalonSRX(3)

        # Enable auto breaking
        self.frontLeftTalon.setNeutralMode(NeutralMode.Brake)
        self.rearLeftTalon.setNeutralMode(NeutralMode.Brake)
        self.frontRightTalon.setNeutralMode(NeutralMode.Brake)
        self.rearRightTalon.setNeutralMode(NeutralMode.Brake)

        # Setup encoders
        self.frontLeftTalon.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.rearLeftTalon.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.frontRightTalon.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.rearRightTalon.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        # Setup encoders
        self.leftEncoder = self.rearLeftTalon
        self.rightEncoder = self.rearRightTalon

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        pass

    def teleopInit(self):
        self.joystick_drive = joystick_drive(self.joystick, self.speed, self.ySpeed, self.tSpeed)

    def teleopPeriodic(self):
       
        # Todo: two classes
        # - joystickDrive class: deals with drive/port 0 input depending on controller type AND resolves different styles of axis control
        #   - Ideally, these would return values directly to leftSpeed/rightSpeed as seen bellow
        # - joystickOperator class: deal with operator/port 1 input depending on controller type

        # Calculate right and left speeds
        leftSpeed, rightSpeed = self.joystick_drive.calculateSpeeds(self.joystick)

        # Update Motors
        self.frontLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.rearLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.frontRightTalon.set(ControlMode.PercentOutput, rightSpeed)
        self.rearRightTalon.set(ControlMode.PercentOutput, rightSpeed)

        # Update SmartDashboard
        self.sd.putNumber("Left Encoder", self.leftEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))
        self.sd.putNumber("Right Encoder", self.rightEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))

if __name__ == "__main__":
    wpilib.run(Robot)
