#!/usr/bin/env python3
import wpilib
from wpilib import ADXRS450_Gyro
from networktables import NetworkTables

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice
import pathfinder as pf
import math

class Robot(wpilib.TimedRobot):
    WHEEL_CIRCUMFERENCE = 0.1524 * math.pi # meters (6 inches)
    DRIVE_WIDTH = 0.305473061 # meters (23 inches)
    ENCODER_COUNTS_PER_REV = 4096
    KP = 0.028
    KV = 3.743381
    KA = 0.717 #22.355086
    KS = 1.08
    DT = 0.02

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

        self.turnSlowdown = 0.8
        
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

        pVal = 0.3
        dVal = 0.6

        self.frontLeftTalon.config_kP(0, pVal, self.kTimeoutMs)
        self.frontRightTalon.config_kP(0, pVal, self.kTimeoutMs)
        self.rearLeftTalon.config_kP(0, pVal, self.kTimeoutMs)
        self.rearRightTalon.config_kP(0, pVal, self.kTimeoutMs)

        self.frontLeftTalon.config_kI(0, 0.000, 0)
        self.frontRightTalon.config_kI(0, 0.000, 0)
        self.rearLeftTalon.config_kI(0, 0.000, 0)
        self.rearRightTalon.config_kI(0, 0.000, 0)

        self.frontLeftTalon.config_kD(0, dVal, 0)
        self.frontRightTalon.config_kD(0, dVal, 0)
        self.rearLeftTalon.config_kD(0, dVal, 0)
        self.rearRightTalon.config_kD(0, dVal, 0)

        # Setup encoders
        self.leftEncoder = self.rearLeftTalon
        self.rightEncoder = self.rearRightTalon

        # Setup Gyro
        self.gyro = ADXRS450_Gyro()

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        # Reset Encoders
        self.leftEncoder.setSelectedSensorPosition(0, 0, 10)
        self.rightEncoder.setSelectedSensorPosition(0, 0, 10)

        # Reset Gyro
        self.gyro.reset()

        # Set up the trajectory
        points = [pf.Waypoint(0,0,0),
                  pf.Waypoint(10, 0, 0)]

        trajectory = pf.generator.generate_trajectory(
            points,
            pf.hermite.pf_fit_hermite_cubic,
            pf.SAMPLES_FAST,
            dt=self.DT, #self.getPeriod(),
            max_velocity=self.KV,
            max_acceleration=self.KA,
            max_jerk=60
        )

        # Wheelbase Width = 2 ft
        left, right = pf.modifiers.tank(trajectory, self.DRIVE_WIDTH)

        # Do something with the new Trajectories...
        leftFollower = pf.followers.EncoderFollower(left, self.logger)
        leftFollower.configureEncoder(
            self.leftEncoder.getSelectedSensorPosition(self.kPIDLoopIdx), self.ENCODER_COUNTS_PER_REV, self.WHEEL_CIRCUMFERENCE
        )
        leftFollower.configurePIDVA(0, 0.0, 0.0, 1, 1)

        rightFollower = pf.followers.EncoderFollower(right, self.logger)
        rightFollower.configureEncoder(
            -self.rightEncoder.getSelectedSensorPosition(self.kPIDLoopIdx), self.ENCODER_COUNTS_PER_REV, self.WHEEL_CIRCUMFERENCE
        )
        rightFollower.configurePIDVA(0, 0.0, 0.0, 1, 1)

        self.leftFollower = leftFollower
        self.rightFollower = rightFollower
    
    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        leftSpeed = self.leftFollower.calculate(self.leftEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))
        rightSpeed = self.rightFollower.calculate(self.rightEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))

        gyro_heading = (
            -self.gyro.getAngle()
        )  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(
            self.leftFollower.getHeading()
        )  # Should also be in degrees

        # This is a poor man's P controller
        angleDifference = pf.bound_angle(desired_heading - gyro_heading)
        turn = self.KS * (-1.0 / 80.0) * angleDifference

        leftSpeed = leftSpeed # + turn
        rightSpeed = rightSpeed # - turn

        self.sd.putNumber("Delta speed", leftSpeed - rightSpeed)
        

        self.velocity = -self.leftEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx) + self.rightEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx)

        self.sd.putNumber("Right vel", self.rightEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx))
        self.sd.putNumber("Left vel", -self.leftEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx))

        
        
        # m/s leftSpeed * self.KV
        # m/s / encoder counts
        #leftVelocity = leftSpeed * self.ENCODER_COUNTS_PER_REV * self.WHEEL_CIRCUMFERENCE
        #rightVelocity = rightSpeed * self.ENCODER_COUNTS_PER_REV * self.WHEEL_CIRCUMFERENCE

        leftVelocity = (leftSpeed*self.ENCODER_COUNTS_PER_REV) / (self.WHEEL_CIRCUMFERENCE*10)
        rightVelocity = (rightSpeed*self.ENCODER_COUNTS_PER_REV) / (self.WHEEL_CIRCUMFERENCE*10)
        
        self.sd.putNumber("Target Right vel", rightVelocity)
        self.sd.putNumber("Target Left vel", leftVelocity)

        self.frontLeftTalon.set(ControlMode.Velocity, leftVelocity)
        self.rearLeftTalon.set(ControlMode.Velocity, leftVelocity)
        self.frontRightTalon.set(ControlMode.Velocity, rightVelocity)
        self.rearRightTalon.set(ControlMode.Velocity, rightVelocity)


    def teleopInit(self):
        # Reset Gyro
        self.gyro.reset()

        self.maxSpeed = 0
        self.last_velocity = 0
        self.maxAccel = 0

    def teleopPeriodic(self):
        # Get max speed
        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

        # Get turn and movement speeds
        self.yAxis = self.threshhold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed
        self.tAxis = self.threshhold(self.joystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed * (1 - abs(self.yAxis) * self.turnSlowdown) 
       
        
        # Calculate right and left speeds
        leftSpeed = self.yAxis+self.tAxis
        rightSpeed = self.yAxis-self.tAxis


        # debouncering button and emergency braking
        if self.joystick.getRawButton(1):
            # print("brake")
            leftSpeed = 0
            rightSpeed = 0

        # Update Motors
        self.frontLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.rearLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.frontRightTalon.set(ControlMode.PercentOutput, rightSpeed)
        self.rearRightTalon.set(ControlMode.PercentOutput, rightSpeed)

        self.velocity = -self.leftEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx) + self.rightEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx)

        self.velocity = self.velocity / 2

        self.sd.putNumber("Velocity", self.velocity / self.ENCODER_COUNTS_PER_REV * self.WHEEL_CIRCUMFERENCE * 10)

        self.acceleration = (self.velocity - self.last_velocity) / self.DT

        # Update SmartDashboard
        # self.sd.putNumber("Velocity", self.velocity)
        # self.sd.putNumber("Right Velocity", self.rightEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx))
        # self.sd.putNumber("Left Velocity", self.leftEncoder.getSelectedSensorVelocity(self.kPIDLoopIdx))

        if self.acceleration > self.maxAccel:
            self.maxAccel = self.acceleration

        # self.sd.putNumber("Acceleration", self.acceleration)
        self.sd.putNumber("Max Acceleration", self.maxAccel/self.ENCODER_COUNTS_PER_REV * self.WHEEL_CIRCUMFERENCE * 10)

        # self.sd.putNumber("Turn Slowdown", (1 - self.yAxis * self.turnSlowdown))

        if self.velocity > self.maxSpeed:
            self.maxSpeed = self.velocity
        
        self.sd.putNumber("Max Velocity", self.maxSpeed/self.ENCODER_COUNTS_PER_REV * self.WHEEL_CIRCUMFERENCE * 10)
        self.last_velocity = self.velocity

if __name__ == "__main__":
    wpilib.run(Robot)
