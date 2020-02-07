#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice
from intake import Intake


class Robot(wpilib.TimedRobot):
    def threshold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)

    def encoderCalculate(self, encoder, unit):
        ticks = encoder.getSelectedSensorPosition(self.kPIDLoopIdx)
        ticks = float(ticks)
        if unit == 'm':
            return ticks/8554.72
        elif unit == 'in':
            return ticks/217.3

    # def calculateVelocity(self, encoder1, encoder2):
        
    #     self.previousAvg = self.currentAvg
    #     self.currentAvg = (self.encoderCalculate(encoder1, 'in') + self.encoderCalculate(encoder2, 'in')) / 2

    #     distance = self.currentAvg - self.previousAvg

    #     # is meters per second
    #     velocity = distance * 50

    #     return velocity 


    def calculateVelocity(self, motor1, motor2, motor3, motor4):
        velocity = (motor1.get() + motor2.get() + motor3.get() + motor4.get()) / 4
        return velocity

    def calculateIntakeSpeed(self, velocity):
        if (velocity < 0):
            return self.baseIntakeSpeed + abs(velocity)
        else:
            return self.baseIntakeSpeed


    def robotInit(self):
        self.kSlotIdx = 0
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        
		# Sets the speed
        self.speed = 0.4
        self.ySpeed = 1
        self.tSpeed = 0.75
        self.baseIntakeSpeed = 5

        self.previousAvg = 0
        self.currentAvg = 0

        
        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')
        
        # joysticks 1 on the driver station
        self.button = ButtonDebouncer(wpilib.Joystick(0), 0)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()

        # TODO: Fix module number
        self.compressor = wpilib.Compressor()
        
        # Talon CAN devices
        self.frontLeftTalon = WPI_TalonSRX(2)
        self.rearLeftTalon = WPI_TalonSRX(0)
        self.frontRightTalon = WPI_TalonSRX(3)
        self.rearRightTalon = WPI_TalonSRX(1)

        # Intake Device
        self.mainIntake = Intake(0,0,0,0,0,0,0)
        self.intakeButton = 1

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
        self.compressor.start()
        # self.timer.start()

        self.frontLeftTalon.set(ControlMode.Speed)
        self.frontRightTalon.set(ControlMode.Speed)
        self.rearLeftTalon.set(ControlMode.Speed)
        self.rearRightTalon.set(ControlMode.Speed)

        pass 

    def teleopPeriodic(self):
        
        # Get max speed
        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

        # Intake Speed 
        self.mainIntake.speed(self.calculateIntakeSpeed(self.calculateVelocity(self.frontLeftTalon, self.frontRightTalon, self.rearLeftTalon, self.rearRightTalon)))

        # Intake Control
        if (self.joystick.getRawButtonPressed(self.intakeButton)):
            self.mainIntake.extend()
        else:
            self.mainIntake.retract()

        # Get turn and movement speeds
        self.tAxis = self.threshold(self.joystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed
        self.yAxis = self.threshold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed
        
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
