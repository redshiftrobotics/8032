#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

from follow_trajectory import TrajectoryFollower

class Robot(wpilib.TimedRobot):
    def threshhold(self, value, limit):
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
        
        # Stores turn and movement axis
        self.yAxis = 0
        self.tAxis = 0

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # joysticks 1 on the driver station
        self.joystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()
        
        # Talon CAN devices
        self.frontLeftTalon = WPI_TalonSRX(2)
        self.rearLeftTalon = WPI_TalonSRX(0)
        self.frontRightTalon = WPI_TalonSRX(3)
        self.rearRightTalon = WPI_TalonSRX(1)

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

        self.trajectory_follower = TrajectoryFollower(self.leftEncoder, self.rightEncoder)

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.trajectory_follower.follow_trajectory("turn")
    
    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        if self.trajectory_follower.is_following("turn"):
            # Update Motors
            leftSpeed, rightSpeed = self.trajectory_follower.run()
            self.frontLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
            self.rearLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
            self.frontRightTalon.set(ControlMode.PercentOutput, rightSpeed)
            self.rearRightTalon.set(ControlMode.PercentOutput, rightSpeed)
        else:
            self.frontLeftTalon.set(ControlMode.PercentOutput, 0)
            self.rearLeftTalon.set(ControlMode.PercentOutput, 0)
            self.frontRightTalon.set(ControlMode.PercentOutput, 0)
            self.rearRightTalon.set(ControlMode.PercentOutput, 0)


    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Get max speed
        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

        # Get turn and movement speeds
        self.tAxis = self.threshhold(self.joystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed
        self.yAxis = self.threshhold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed
        
        # Calculate right and left speeds
        leftSpeed = self.yAxis+self.tAxis
        rightSpeed = self.yAxis-self.tAxis

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
