#!/usr/bin/env python3
import wpilib
from wpilib import DriverStation
from wpilib import SendableChooser
from wpilib import SmartDashboard
from wpilib import CameraServer
from wpilib import ADXRS450_Gyro
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice, FollowerType, FeedbackDevice
import math
import config
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
                steeringAdjust = config.limeLight.aimKp * headingError#- config.limeLight.minAimCommand
            elif tx < 1.0:
                steeringAdjust = config.limeLight.aimKp * headingError#+ config.limeLight.minAimCommand

            distanceAdjust = distanceKp * distanceError
        else:
            steeringAdjust = 10.0 * config.limeLight.aimKp

        return distanceAdjust, steeringAdjust

    def robotInit(self):
        

        # Smart Dashboard
        self.autoTabSD = NetworkTables.getTable('Auto')
        self.teleopTabSD = NetworkTables.getTable('Teleop')
        self.debugTabSD = NetworkTables.getTable('Debug')

        # joystick 0 on the driver station
       
        self.driverJoystickLeft = wpilib.Joystick(config.joystick.driverJoystickLeftNumber)
        self.driverJoystickRight = wpilib.Joystick(config.joystick.driverJoystickRightNumber)
        config.joystick.operatorJoystick = wpilib.Joystick(config.joystick.operatorJoystick)
       
        # Create a simple timer
        self.timer = wpilib.Timer()

        # Setup the compressor
        self.compressor = wpilib.Compressor(0)

        # Setup the intake
        self.intake = Intake(7,0,1,0)
        self.intakeToggle = ButtonDebouncer(config.joystick.operatorJoystick, 6) # [Right Bumper]
        self.intakeCollectToggle = ButtonDebouncer(config.joystick.operatorJoystick, 4) # [A Button]
        config.intake.intakeCollectToggleOn = False
    
        # Setup the transit
        self.transit = Transit(config.transit.port)

        # Setup the hang
        self.hang = Hang(config.hang.leftMotor_ID, config.hang.rightMotor_ID, config.hang.stopPoint, config.hang.maxSpeed)
      
        # Setup Master motors for each side
        self.leftMaster = WPI_TalonSRX(config.motors.leftMaster) # Front left Motor
        self.leftMaster.setInverted(True)
        self.leftMaster.setSensorPhase(True)
        self.leftMaster.setNeutralMode(NeutralMode.Brake)

        self.rightMaster = WPI_TalonSRX(config.motors.rightMaster) # Front right Motor
        self.rightMaster.setInverted(False)
        self.rightMaster.setSensorPhase(True)
        self.rightMaster.setNeutralMode(NeutralMode.Brake)

        # Setup Follower motors for each side
        self.leftAlt = WPI_TalonSRX(config.motors.leftAlt) # Back left motor
        self.leftAlt.setInverted(True)
        self.leftAlt.follow(self.leftMaster)
        self.leftAlt.setNeutralMode(NeutralMode.Brake)

        self.rightAlt = WPI_TalonSRX(config.motors.rightAlt) # Back right motor
        self.rightAlt.setInverted(False)
        self.rightAlt.follow(self.rightMaster)
        self.rightAlt.setNeutralMode(NeutralMode.Brake)

        # Setup encoders
        self.leftMaster.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            config.MISC.kPIDLoopIdx,
            config.MISC.kTimeoutMs,
        )

        self.rightMaster.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative,
            config.MISC.kPIDLoopIdx,
            config.MISC.kTimeoutMs,
        )

        # Setup Gyro
        self.gyro = ADXRS450_Gyro()

        # Start camera server
        # NOTE: cscore needs to be manually installed on the roborio for this to work
        CameraServer.launch()

       

        self.leftMaster.config_kP(config.MISC.kPIDLoopIdx,config.motors.kP,config.MISC.kTimeoutMs)
        self.leftMaster.config_kD(config.MISC.kPIDLoopIdx,config.motors.kD,config.MISC.kTimeoutMs)
        self.leftMaster.config_kI(config.MISC.kPIDLoopIdx,config.motors.kI,config.MISC.kTimeoutMs)

        self.rightMaster.config_kP(config.MISC.kPIDLoopIdx,config.motors.kP,config.MISC.kTimeoutMs)
        self.rightMaster.config_kD(config.MISC.kPIDLoopIdx,config.motors.kD,config.MISC.kTimeoutMs)
        self.rightMaster.config_kI(config.MISC.kPIDLoopIdx,config.motors.kI,config.MISC.kTimeoutMs)

        # Setup Differential Drive
        self.drive = Drive(self.leftMaster, self.rightMaster, self.rightAlt, self.leftAlt)

        # Setup Limelight
        self.limelight = LimeLight()
       

        config.auto.initiationLineDistance /= self.ENCODER_CONSTANT

        # Create ShuffleBoard Widgets
        self.debugTabSD.putBoolean("Retract Hang", False)
        
        self.autoTabSD.putNumber("Auto Wait Time", 0)
        self.autoTabSD.putString("Auto State", "wait")
        self.autoTabSD.putNumber("Left Encoder", 0)
        self.autoTabSD.putNumber("Left Target", 0)
        self.autoTabSD.putNumber("Right Encoder", 0)
        self.autoTabSD.putNumber("Right Target", 0)
        self.autoTabSD.putNumber("Missed Frames", 0)
        self.autoTabSD.putBoolean("Power Port Visible", 0)

        self.autoSelector = SendableChooser()
        self.autoSelector.addOption("IL Power Port", AUTOS["initiation-line"]["power-port"])
        self.autoSelector.addOption("IL Shield Generator", AUTOS["initiation-line"]["shield-generator"])
        self.autoSelector.addOption("3 Ball", AUTOS["3-ball"])
        self.autoSelector.setDefaultOption("3 Ball", AUTOS["3-ball"])
        SmartDashboard.putData(self.autoSelector)


    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        # Setup auto state
        self.selectedAuto = self.autoSelector.getSelected()
        self.waitTime = self.autoTabSD.getNumber("Auto Wait Time", 0)
        self.autoState = "wait"
        self.timer.reset()
        self.timer.start()

        # Setup Encoders
        self.leftMaster.setSelectedSensorPosition(0, config.MISC.kPIDLoopIdx, config.MISC.kTimeoutMs)
        self.rightMaster.setSelectedSensorPosition(0, config.MISC.kPIDLoopIdx, config.MISC.kTimeoutMs)

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_ON)
        self.missedFrames = 0

        # Setup the compressor
        self.compressor.start()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""

        # Stop all unused mechanisms
        self.drive.stop()
        self.intake.stop()
        self.transit.stop()
        
        # Run the selected auto
        if self.selectedAuto == AUTOS["initiation-line"]["power-port"] or self.selectedAuto == AUTOS["initiation-line"]["shield-generator"]:
            if self.autoState == "wait":
                if self.timer.get() < self.waitTime:
                    pass
                else:
                    self.autoState = "drive"
            elif self.autoState == "drive":
                moveTgt = config.auto.initiationLineDistance
                if self.selectedAuto == AUTOS["initiation-line"]["power-port"]:
                    moveTgt *= 1
                elif self.selectedAuto == AUTOS["initiation-line"]["shield-generator"]:
                    moveTgt *= -1
                if abs(self.drive.getEncoderAverage()) < abs(moveTgt):
                    self.drive.tankDrive(moveTgt, moveTgt, ControlMode.Position)
                else:
                    self.autoState = "stop"
            elif self.autoState == "stop":
                self.drive.stop()
        
        elif self.selectedAuto == AUTOS["3-ball"]:
            if self.autoState == "wait":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                if self.timer.get() < self.waitTime:
                    pass
                else:
                    self.autoState = "align"
            elif self.autoState == "align":
                self.limelight.setLedMode(LIMELIGHT_LED_ON)
                if self.limelight.getTV() > 0:
                    distError = self.limelight.getDistance(self.targetBottomHeight)
                    if distError is not None:
                        distError -= config.limeLight.targetDistance
                        angleError = self.limelight.getTX()

                        # Check if the robot is aligned
                        if ((distError < config.limeLight.distanceThreshold) and (angleError < self.angleThreshold)):
                            self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                            self.timer.reset()
                            self.autoState = "move"
                        # If not continue aligning
                        else:
                            x, t = self.align(config.limeLight.targetDistance, self.targetBottomHeight)
                            self.drive.arcadeDrive(x, t, ControlMode.PercentOutput)
                else:
                    self.missedFrames += 1
                if (self.missedFrames >= 75):
                    self.timer.reset()
                    self.autoState = "move"
            elif self.autoState == "move":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                if self.timer.get() < config.auto.moveTime:
                    self.drive.arcadeDrive(-0.7, 0.2, ControlMode.PercentOutput)
                else:
                    self.timer.reset()
                    self.autoState = "deposit"
            elif self.autoState == "deposit":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                if self.timer.get() < config.auto.depositTime:
                    self.transit.backward()
                else:
                    self.timer.reset()
                    self.autoState = "leave"
            elif self.autoState == "leave":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                if self.timer.get() < config.auto.leaveTime:
                    self.drive.tankDrive(0.2, 0.6, ControlMode.PercentOutput)
                else:
                    self.autoState = "back"
                    self.timer.reset()
            elif self.autoState == "back":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                if self.timer.get() < config.auto.backwardsTime:
                    self.drive.arcadeDrive(0.5, 0, ControlMode.PercentOutput)
                else:
                    self.timer.reset()
                    self.autoState = "stop"
            elif self.autoState == "stop":
                self.limelight.setLedMode(LIMELIGHT_LED_OFF)
                self.drive.stop()
        
        self.drive.update()
        self.intake.update()
        self.transit.update()

        self.autoTabSD.putString("Auto State", self.autoState)
        self.autoTabSD.putNumber("Left Encoder", self.drive.getLeftEncoder())
        self.autoTabSD.putNumber("Left Target", self.drive.leftSpeed)
        self.autoTabSD.putNumber("Right Encoder", self.drive.getRightEncoder())
        self.autoTabSD.putNumber("Right Target", self.drive.rightSpeed)
        self.autoTabSD.putNumber("Missed Frames", self.missedFrames)
        self.autoTabSD.putBoolean("Power Port Visible", self.limelight.getTV())

    def teleopInit(self):
        """Called only at the beginning of teleop mode."""
        # Setup the compressor
        self.compressor.start()

        # Turn off the limelight LED
        self.limelight.setLedMode(LIMELIGHT_LED_OFF)

        # Reset ShuffleBoard Widgets
        self.debugTabSD.putBoolean("Retract Hang", False)

    def teleopPeriodic(self):
        """Called every 20ms in autonomous mode."""

        # INTAKE CODE
        # Extend the intake if needed
        if self.intakeToggle.get():
            self.intake.toggle()
        
        # Keep track of the intake toggle state
        if self.intakeCollectToggle.get():
           config.intake.intakeCollectToggleOn = not config.intake.intakeCollectToggleOn
        # Update the direction of the intake based on whether the forward toggle is on, 
        #   or the back button is pressed
        # Update the speed of the intake based on the driving direction of the robot
        if config.joystick.operatorJoystick.getRawButton(config.intake.intakeReverse):
            self.intake.speed(-config.intake.baseIntakeSpeed)
            self.intake.enable_collect()
        elif config.intake.intakeCollectToggleOn:
           robotDirection = self.leftMaster.get() + self.rightMaster.get()
           if robotDirection < 0:
               self.intake.speed(config.intake.baseIntakeSpeed+0.5)
           else:
               self.intake.speed(config.intake.baseIntakeSpeed)
           self.intake.enable_collect()
        else:
            self.intake.disable_collect()
        
        # TRANSIT CODE
        # Update the transit speed based on joystick
        self.transit.speed(self.threshold(config.joystick.operatorJoystick.getRawAxis(config.transit.transitAxis), 0.05))

        # Update transit based on D-PAD for indexing
        rightPOV = config.joystick.operatorJoystick.getPOV(0)
        if rightPOV == 0:
            self.transit.speed(self.transitIndexSpeed)
        elif rightPOV == 180:
            self.transit.speed(-self.transitIndexSpeed)

        # HANG CODE
        # Update each side of the hang to move based on the corresponding joystick trigger
        self.hang.move(config.joystick.operatorJoystick.getRawAxis(config.hang.hangLeftAxis), config.joystick.operatorJoystick.getRawAxis(config.hang.hangRightAxis))

        # Retract the hang if the corresponding button on ShuffleBoard is pressed
        if self.autoTabSD.getBoolean("Retract Hang", False):
            self.hang.retract()

        # DRIVE CODE
        # Check if stop robot button is pressed
        if self.driverJoystickRight.getRawButton(config.joystick.stopButton):
            self.drive.arcadeDrive(0,0, ControlMode.PercentOutput)
        else:
            # Set the max speed of the robot
            if self.driverJoystickRight.getRawButton(config.joystick.halfSpeedButton):
                config.speed.speed = 0.5
            else:
                config.speed.speed = 1.0

            # Get turn and movement speeds
            tAxis = -self.threshold(self.driverJoystickLeft.getRawAxis(0), 0.05) * config.speed.tSpeed * config.speed.speed
            xAxis = -self.threshold(self.driverJoystickRight.getRawAxis(1), 0.05) * config.speed.xSpeed * config.speed.speed
            self.drive.arcadeDrive(xAxis, tAxis, ControlMode.PercentOutput)

        # UPDATE CODE
        self.drive.update()
        self.intake.update()
        self.transit.update()
        self.hang.update()
        # NOTE: If auto-alignment to the loading bay is added, this will need to be moved so that the LED can turn on during use
        self.limelight.setLedMode(LIMELIGHT_LED_OFF)

        self.teleopTabSD.putNumber("Balls", 0)
        #self.teleopTabSD.putNumber("Gyro", self.gyro.getAngle())


AUTOS = {
    "initiation-line": {
        "power-port": 0,
        "shield-generator": 1
    },
    "3-ball": 2
}

if __name__ == "__main__":
    wpilib.run(Robot)
