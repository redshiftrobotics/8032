#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice, FollowerType, FeedbackDevice
import math

from intake import Intake
from transit import Transit
from hang import Hang
from drive import Drive
from limelight import LimeLight, LIMELIGHT_LED_OFF, LIMELIGHT_LED_ON

class Robot(wpilib.TimedRobot):
    WHEEL_CIRCUMFERENCE = 0.1524 * math.pi # meters (6 inches)
    ENCODER_COUNTS_PER_REV = 4096
    ENCODER_CONSTANT = (1/ENCODER_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE

    def threshold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return value


    def align(self, targetDistance, targetHeight):
        """Generates turn and move values based on a target distance and height"""
        tv = self.limelight.getTV()
        tx = self.limelight.getTX() + 5
        ta = self.limelight.getTA()
        ts = self.limelight.getTS()

        steeringAdjust = 0.0
        distanceAdjust = 0.0

        if self.limelight.getTV() > 0.0:
            headingError = -tx
            distanceError = targetDistance - self.limelight.getDistance(targetHeight)

            if tx > 1.0: 
                steeringAdjust = self.aimKp * headingError#- self.minAimCommand
            elif tx < 1.0:
                steeringAdjust = self.aimKp * headingError#+ self.minAimCommand

            distanceAdjust = self.distanceKp * distanceError
        else:
            steeringAdjust = 10.0 * self.aimKp

        return distanceAdjust, steeringAdjust

    def robotInit(self):
        self.kSlotIdx = 0
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        
        # Sets the speed
        self.speed = 1.0
        self.xSpeed = 1.0
        self.tSpeed = 2.0/3.0

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # joystick 0 on the driver station
        self.driverJoystickLeft = wpilib.Joystick(0)
        self.driverJoystickRight = wpilib.Joystick(1)
        self.operatorJoystick = wpilib.Joystick(2)
        self.halfSpeedButton = 2
        self.stopButton = 1
        
        # Create a simple timer
        self.timer = wpilib.Timer()

        # Setup the compressor
        self.compressor = wpilib.Compressor(0)

        # Setup the intake
        self.intake = Intake(7,0,1,0)
        self.intakeToggle = ButtonDebouncer(self.operatorJoystick, 6)
        self.intakeCollectToggle = ButtonDebouncer(self.operatorJoystick, 4)
        self.intakeCollectToggleOn = False
        self.intakeReverse = 1
        self.intakeForward = 4
        self.baseIntakeSpeed = 0.4

        # Setup the transit
        self.transit = Transit(6)
        self.transitAxis = 1

        # Setup the hang
        self.hang = Hang(8, 9, 0.1, -0.1, 0)

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
        self.leftAlt = WPI_TalonSRX(2) # Back left motor
        self.leftAlt.setInverted(False)
        self.leftAlt.follow(self.leftMaster)
        self.leftAlt.setNeutralMode(NeutralMode.Brake)

        self.rightAlt = WPI_TalonSRX(3) # Back right motor
        self.rightAlt.setInverted(True)
        self.rightAlt.follow(self.rightMaster)
        self.rightAlt.setNeutralMode(NeutralMode.Brake)

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

        # Setup Talon PID constants
        # TODO: tune PID
        self.kP = 0.125
        self.kD = 0
        self.kI = 0

        self.leftMaster.config_kP(self.kPIDLoopIdx,self.kP,self.kTimeoutMs)
        self.leftMaster.config_kD(self.kPIDLoopIdx,self.kD,self.kTimeoutMs)
        self.leftMaster.config_kI(self.kPIDLoopIdx,self.kI,self.kTimeoutMs)

        self.rightMaster.config_kP(self.kPIDLoopIdx,self.kP,self.kTimeoutMs)
        self.rightMaster.config_kD(self.kPIDLoopIdx,self.kD,self.kTimeoutMs)
        self.rightMaster.config_kI(self.kPIDLoopIdx,self.kI,self.kTimeoutMs)

        # Setup Differential Drive
        self.drive = Drive(self.leftMaster, self.rightMaster, self.rightAlt, self.leftAlt)

        # Setup Limelight
        self.limelight = LimeLight()
        self.targetDistance = 15.0 # in
        self.targetBottomHeight = 89.5 # in
        # TODO: Tune limelight PID
        self.aimKp = 0.02
        self.distanceKp = 0.008
        self.minAimCommand = 0.05
        self.distanceThreshold = 20
        self.angleThreshold = 5

        # Setup auto parameters
        self.waitTime = 1.0
        self.depositTime = 0.75
        self.moveTime = 1.0
        #self.leftLeaveDist = 10_000 # encoder ticks
        #self.rightLeaveDist = 5_000# encoder ticks
        self.leaveTime = 0.7
        self.backwardsTime = 1.0
        self.leaveThreshold = 50

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        # Setup auto state
        self.autoState = "wait"
        self.timer.reset()
        self.timer.start()

        # Setup Encoders
        self.leftMaster.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
        self.rightMaster.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_ON)
        self.missedFrames = 0

        # Setup the compressor
        self.compressor.start()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        self.sd.putNumber("ty", self.limelight.getTY())
        self.sd.putNumber("tx", self.limelight.getTX())
        self.sd.putNumber("tv", self.limelight.getTV())

        dist = self.limelight.getDistance(self.targetBottomHeight)
        if dist:
            self.sd.putNumber("dist", dist)

        # Stop all unused mechanisms
        self.drive.stop()
        self.intake.stop()
        self.transit.stop()
        #self.hang.stop()

        if self.autoState == "wait":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            if self.timer.get() < self.waitTime:
                pass
            else:
                self.autoState = "align"
        elif self.autoState == "align":
            self.limelight.setLedMode(LIMELIGHT_LED_ON)
            if self.limelight.getTV() > 0:
                distError = self.limelight.getDistance(self.targetBottomHeight) - self.targetDistance
                angleError = self.limelight.getTX()

                # Check if the robot is aligned
                if ((distError < self.distanceThreshold) and (angleError < self.angleThreshold)):
                    self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                    self.timer.reset()
                    self.autoState = "move"
                # If not continue aligning
                else:
                    x, t = self.align(self.targetDistance, self.targetBottomHeight)
                    self.drive.arcadeDrive(x, t, ControlMode.PercentOutput)
            else:
                self.missedFrames += 1
            if (self.missedFrames >= 75):
                self.timer.reset()
                self.autoState = "move"
        elif self.autoState == "move":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            if self.timer.get() < self.moveTime:
                self.drive.arcadeDrive(-0.7, 0.2, ControlMode.PercentOutput)
            else:
                self.timer.reset()
                self.autoState = "deposit"
        elif self.autoState == "deposit":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            if self.timer.get() < self.depositTime:
                self.transit.backward()
            else:
                self.timer.reset()
                self.autoState = "leave"
        elif self.autoState == "leave":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            if self.timer.get() < self.leaveTime:
                self.drive.tankDrive(0.33, 1.0, ControlMode.PercentOutput)
            else:
                self.autoState = "back"
                self.timer.reset()
        elif self.autoState == "back":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            if self.timer.get() < self.backwardsTime:
                self.drive.arcadeDrive(0.5, 0, ControlMode.PercentOutput)
            else:
                self.timer.reset()
                self.autoState = "stop"
        elif self.autoState == "stop":
            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
            self.drive.stop()
        
        self.drive.update()
        #self.intake.update()
        self.transit.update()
        #self.hang.update()
        self.sd.putString("state", self.autoState)
        self.sd.putNumber("missed frames", self.missedFrames)
        self.sd.putNumber("R ENC", self.rightMaster.getSelectedSensorPosition())
        self.sd.putNumber("L ENC", self.leftMaster.getSelectedSensorPosition())
        self.sd.putNumber("timer", self.timer.get())

    def teleopInit(self):
        """Called only at the beginning of teleop mode."""
        # Setup the compressor
        self.compressor.start()

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_ON)

    def teleopPeriodic(self):
        """Called every 20ms in autonomous mode."""
        # Extend the intake if needed
        if self.intakeToggle.get():
            self.intake.toggle()
        
        # Keep track of the intake toggle state
        if self.intakeCollectToggle.get():
           self.intakeCollectToggleOn = not self.intakeCollectToggleOn

        # Update the speed of the intake based on whether the forward toggle is on, or the back button is pressed
        if self.operatorJoystick.getRawButton(self.intakeReverse):
            self.intake.speed(-self.baseIntakeSpeed)
            self.intake.enable_collect()
        # elif self.operatorJoystick.getRawButton(self.intakeForward):
        #     self.intake.speed(self.baseIntakeSpeed)
        #     self.intake.enable_collect()
        elif self.intakeCollectToggleOn:
           robotDirection = self.leftMaster.get() + self.rightMaster.get()
           if robotDirection < 0:
               self.intake.speed(self.baseIntakeSpeed+0.5)
           else:
               self.intake.speed(self.baseIntakeSpeed)
           self.intake.enable_collect()
        else:
            self.intake.disable_collect()
        
        # Update the transit speed
        self.transit.speed(self.threshold(self.operatorJoystick.getRawAxis(self.transitAxis), 0.05))

        # Get max speed
        #self.speed = (-self.driverJoystickLeft.getRawAxis(3) + 1)/2
        
        if self.driverJoystickRight.getRawButton(self.stopButton):
            self.drive.arcadeDrive(0,0, ControlMode.PercentOutput)
        else:
            if self.driverJoystickRight.getRawButton(self.halfSpeedButton):
                self.speed = 0.5
            else:
                self.speed = 1.0

            # Get turn and movement speeds
            tAxis = -self.threshold(self.driverJoystickLeft.getRawAxis(0), 0.05) * self.tSpeed * self.speed
            xAxis = self.threshold(self.driverJoystickRight.getRawAxis(1), 0.05) * self.xSpeed * self.speed
            self.drive.arcadeDrive(xAxis, tAxis, ControlMode.PercentOutput)

        self.drive.update()
        self.intake.update()
        self.transit.update()
        #self.hang.update()
        self.limelight.setLedMode(LIMELIGHT_LED_OFF)
        
    
if __name__ == "__main__":
    wpilib.run(Robot)
