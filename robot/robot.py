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
    def threshold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)


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
        self.baseIntakeSpeed = 0.4

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

        # Setup the transit
        self.transit = Transit(6)
        self.transitForward = 3
        self.transitBackward = 4

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

        # Setup Differential Drive
        self.drive = Drive(self.leftMaster, self.rightMaster, self.rightAlt, self.leftAlt)
        
        # Setup Limelight
        self.limelight = LimeLight()
        self.targetDistance = 4.0
        self.targetBottomHeight = 81.0 # in
        self.aimKp = 0.05
        self.distanceKp = 0.25
        self.minAimCommand = 0.05
        self.distanceThreshold = 0.1
        self.angleThreshold = 1

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.autoState = "align"
        self.limelight.setLedMode(LIMELIGHT_LED_ON)

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
        if self.autoState == "align":
            distError = self.limelight.getDistance(self.targetBottomHeight) - self.targetDistance
            angleError = self.limelight.getTX()

            if (self.threshold(distError, self.distanceThreshold) == 0) and self.threshold(angleError, self.angleThreshold):
                self.autoState == "move"
                # Reset encoders
            else:
                self.align(self.targetDistance, self.targetBottomHeight)
        elif self.autoState == "move":
            # Insert code to move with encoders here
            # Average and check if encoder is equal to the desired state
            # When done, reset the timer
            pass
        elif self.autoState == "deposit":
            # Run run deposit until timer is done
            pass
        elif self.autoState == "leave":
            

    def teleopInit(self):
        """Called only at the beginning of teleop mode."""
        self.compressor.start()
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

        self.transit.update()
        self.intake.update()

        #self.hang.update()
        
        self.drive.arcadeDrive(self.xAxis, self.tAxis, ControlMode.PercentOutput)
    
if __name__ == "__main__":
    wpilib.run(Robot)
