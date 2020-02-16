#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice, FollowerType, FeedbackDevice

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
        tx = self.limelight.getTX()
        ta = self.limelight.getTA()

        headingError = -tx
        distanceError = targetDistance - self.limelight.getDistance(targetHeight)
        steeringAdjust = 0.0
        distanceAdjust = 0.0

        if tv == 0.0:
            steeringAdjust = 10.0 * self.aimKp
        else:
            if tx > 1.0: 
                steeringAdjust = self.aimKp * headingError - self.minAimCommand
            elif tx < 1.0:
                steeringAdjust = self.aimKp * headingError + self.minAimCommand

        distanceAdjust = self.distanceKp * distanceError

        return (steeringAdjust, distanceAdjust)

    def robotInit(self):
        self.kSlotIdx = 0
        self.kPIDLoopIdx = 0
        self.kTimeoutMs = 10
        
        # Sets the speed
        self.speed = 0.4
        self.xSpeed = 1
        self.tSpeed = 0.75
        self.turnSlowdown = 0.8

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # joystick 1 on the driver station
        self.driverJoystick = wpilib.Joystick(0)
        
        # Create a simple timer
        self.timer = wpilib.Timer()

        # Setup the compressor
        self.compressor = wpilib.Compressor(61)

        # Setup the intake
        self.intake = Intake(7,61,1,0)
        self.intakeToggle = ButtonDebouncer(self.driverJoystick, 2)
        self.intakeCollect = 1
        self.intakeReverse = 5
        self.baseIntakeSpeed = 0.4

        # Setup the transit
        self.transit = Transit(6)
        self.transitForward = 3
        self.transitBackward = 4

        # Setup the hang
        self.hang = Hang(0, 1, 0.1, -0.1, 0)

        # Setup Master motors for each side
        self.leftMaster = WPI_TalonSRX(4) # Front left Motor
        self.leftMaster.setInverted(False)
        self.leftMaster.setSensorPhase(False)
        self.leftMaster.setNeutralMode(NeutralMode.Brake)

        self.rightMaster = WPI_TalonSRX(5) # Front right Motor
        self.rightMaster.setInverted(False)
        self.rightMaster.setSensorPhase(False)
        self.rightMaster.setNeutralMode(NeutralMode.Brake)

        # Setup Follower motors for each side
        self.leftAlt = WPI_TalonSRX(2) # Back left motor
        self.leftAlt.setInverted(False)
        self.leftAlt.follow(self.leftMaster)
        self.leftAlt.setNeutralMode(NeutralMode.Brake)

        self.rightAlt = WPI_TalonSRX(3) # Back right motor
        self.rightAlt.setInverted(False)
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

        self.leftMaster.config_kP(self.kP)
        self.leftMaster.config_kD(self.kD)
        self.leftMaster.config_kI(self.kI)

        self.rightMaster.config_kP(self.kP)
        self.rightMaster.config_kD(self.kD)
        self.rightMaster.config_kI(self.kI)

        # Setup Differential Drive
        self.drive = Drive(self.leftMaster, self.rightMaster, self.rightAlt, self.leftAlt)

        # Setup Limelight
        self.limelight = LimeLight()
        self.targetDistance = 4.0
        self.targetBottomHeight = 81.0 # in
        # TODO: Tune limelight PID
        self.aimKp = 0.05
        self.distanceKp = 0.25
        self.minAimCommand = 0.05
        # TODO: find accpetable error
        self.distanceThreshold = 0.1
        self.angleThreshold = 1

        # Setup auto parameters
        self.waitTime = 1.0
        # TODO: find optimal deposit time
        self.depositTime = 2.0
        # TODO: find correct distances
        self.leftLeaveDist = self.ENCODER_CONSTANT * 10 # in -> encoder ticks
        self.rightLeaveDist = self.ENCODER_CONSTANT * 40 # in -> encoder ticks
        # TODO: find acceptable error
        self.leaveThreshold = 50

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        # Setup auto state
        self.autoState = "wait"
        self.time.reset()

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_OFF)

        # Setup the compressor
        self.compressor.start()
        self.compressor.clearAllPCMStickyFaults()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        #self.sd.putNumber("tx", self.limelight.getTX())
        #self.sd.putNumber("ta", self.limelight.getTA())
        #self.sd.putNumber("tv", self.limelight.getTV())
        #self.sd.putNumber("ty", self.limelight.getTY())
        #dist = self.limelight.getDistance(self.targetBottomHeight)
        #if dist:
        #    self.sd.putNumber("dist", dist)
        #self.align(self.targetDistance, self.targetBottomHeight)

        # Stop all unused mechanisms
        self.drive.stop()
        self.intake.stop()
        self.transit.stop()
        #self.hang.stop()

        if self.autoState == "wait":
            if self.timer.get() < self.waitTime:
                pass
            else:
                self.autoState = "align"
                self.limelight.setLedMode(LIMELIGHT_LED_ON)
        elif self.autoState == "align":
            distError = self.limelight.getDistance(self.targetBottomHeight) - self.targetDistance
            angleError = self.limelight.getTX()

            # Check if the robot is aligned
            if (distError < self.distanceThreshold) and (angleError < self.angleThreshold):
                self.autoState == "deposit"
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                self.timer.reset()
            # If not continue aligning
            else:
                x, t = self.align(self.targetDistance, self.targetBottomHeight)
                self.drive.arcadeDrive(x, t)
        elif self.autoState == "deposit":
            if self.timer.get() < self.depositTime:
                self.transit.forward()
            else:
                self.leftMaster.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
                self.rightMaster.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
                self.autoState = "leave"
        elif self.autoState == "leave":
            leftError = self.leftMaster.getSelectedSensorPosition() - self.leftLeaveDist
            rightError = self.rightMaster.getSelectedSensorPosition() - self.rightLeaveDist

            # Check if the robot has gone the correct distance
            if (leftError < self.leaveThreshold) and (rightError < self.leaveThreshold):
                self.autoState == "stop"
            # If not continue moving
            else:
                self.drive.tankDrive(self.leftLeaveDist, self.rightLeaveDist, ControlMode.Position)
        elif self.autoState == "stop":
            self.drive.stop()
        
        self.drive.update()
        self.intake.update()
        self.transit.update()
        #self.hang.update()


    def teleopInit(self):
        """Called only at the beginning of teleop mode."""
        # Setup the compressor
        self.compressor.start()
        self.compressor.clearAllPCMStickyFaults()

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_ON)

    def teleopPeriodic(self):
        """Called every 20ms in autonomous mode."""

        # Get max speed
        self.speed = (-self.driverJoystick.getRawAxis(3) + 1)/2

        # Extend the intake if needed
        if self.intakeToggle.get():
            self.intake.toggle()

        # Set the speed of the intake
        if self.driverJoystick.getRawButton(self.intakeCollect):
            robotDirection = self.leftMaster.get() + self.rightMaster.get()
            
            # Check if the robot is moving forwards or backwards
            if robotDirection < 0:
                self.intake.speed(self.baseIntakeSpeed+0.5)
            else:
                self.intake.speed(self.baseIntakeSpeed)
        elif self.driverJoystick.getRawButton(self.intakeReverse):
            self.intake.speed(-self.baseIntakeSpeed)
        else:
            self.intake.speed(0)
        
        # Check update the transit state
        if self.driverJoystick.getRawButton(self.transitForward):
            self.transit.forward()
            #self.hang.extend()
        elif self.driverJoystick.getRawButton(self.transitBackward):
            self.transit.backward()
            #self.hang.retract()
        else:
            self.transit.stop()
            #self.hang.stop()

        # Get turn and movement speeds
        self.xAxis = self.threshold(self.driverJoystick.getRawAxis(2), 0.05) * self.xSpeed * self.speed # * pow((1-abs(self.tAxis)),0.25)
        self.tAxis = self.threshold(self.driverJoystick.getRawAxis(1), 0.05) * self.tSpeed * self.speed # * (1-abs(self.xAxis)) * self.turnSlowdown
        self.drive.arcadeDrive(self.xAxis, self.tAxis, ControlMode.PercentOutput)

        self.drive.update()
        self.intake.update()
        self.transit.update()
        #self.hang.update()
        
    
if __name__ == "__main__":
    wpilib.run(Robot)
