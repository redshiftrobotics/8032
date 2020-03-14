#!/usr/bin/env python3
import wpilib
from wpilib import ADXRS450_Gyro
from wpilib.drive import DifferentialDrive
if wpilib.RobotBase.isSimulation():
    is_sim = True
    import physics
    import time
else:
    is_sim = False

from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

import pathfinder as pf
import math
import time

class Robot(wpilib.TimedRobot):
    WHEEL_CIRCUMFERENCE = 0.1524 * math.pi # meters (6 inches)
    DRIVE_WIDTH = 0.305473061 # meters (23 inches)
    ENCODER_COUNTS_PER_REV = 4096
    KP = 0.028
    MAX_VELOCITY = 3.743381
    MAX_ACCELERATION = 22.355086
    KS = 1.08
    KV = 5.99
    KA = 0.717
    DT = 0.02
    ANGLE_CONSTANT = 0.01
    ENCODER_CONSTANT = (1/ENCODER_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE

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
        
        # Setup Master motors for each side
        self.leftMaster = WPI_TalonSRX(4) # Back left Motor
        self.leftMaster.setInverted(False)
        self.leftMaster.setSensorPhase(False)
        self.leftMaster.setNeutralMode(NeutralMode.Brake)

        self.rightMaster = WPI_TalonSRX(5) # Back right Motor
        self.rightMaster.setInverted(True)
        self.rightMaster.setSensorPhase(False)
        self.rightMaster.setNeutralMode(NeutralMode.Brake)

        # Setup Follower motors for each side
        self.leftFollower0 = WPI_TalonSRX(2) # Front left motor
        self.leftFollower0.setInverted(False)
        self.leftFollower0.follow(self.leftMaster)
        self.leftFollower0.setNeutralMode(NeutralMode.Brake)

        self.rightFollower0 = WPI_TalonSRX(3) # Front right motor
        self.rightFollower0.setInverted(True)
        self.rightFollower0.follow(self.rightMaster)
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

        # Setup PID controller on the talons
        pVal = 0.3
        dVal = 0.6
        self.leftMaster.config_kP(0, pVal, self.kTimeoutMs)
        self.rightMaster.config_kP(0, pVal, self.kTimeoutMs)

        self.leftMaster.config_kD(0, dVal, 0)
        self.rightMaster.config_kD(0, dVal, 0)
        
        self.leftMaster.config_kI(0, 0.000, 0)
        self.rightMaster.config_kI(0, 0.000, 0)

        # Setup Differential Drive
        self.drive = DifferentialDrive(self.leftMaster, self.rightMaster)
        self.drive.setDeadband(0) # Disable auto joystick thresholding

        # Setup Gyro
        self.gyro = ADXRS450_Gyro()

        # Setup the simulation
        if is_sim:
            self.physics = physics.PhysicsEngine()
            self.last_tm = time.time()
            self.left_motor_sim = wpilib.Talon(1)
            self.right_motor_sim = wpilib.Talon(2)
            self.gyro_sim = wpilib.AnalogInput(2)
    
    if is_sim:
        # TODO: this needs to be builtin
        def robotPeriodic(self):
            now = time.time()
            tm_diff = now - self.last_tm
            self.last_tm = now
            self.physics.update_sim(now, tm_diff)

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        # Reset Gyro
        self.gyro.reset()

        # Reset Encoders
        self.leftMaster.setSelectedSensorPosition(0)
        self.rightMaster.setSelectedSensorPosition(0)

        # NOTE: This is for reading a trajectory exported by PathWeaver
        # Read trajectory from file
        path_name = "5 Ball Auto"
        trajectory = pf.read_from_pathweaver(path_name, __file__)

        # NOTE: This is for generating a custom trajectory
        # Set up the trajectory
        # points = [pf.Waypoint(0,0,0),
        #           pf.Waypoint(1, 1, 0)]

        # trajectory = pf.generator.generate_trajectory(
        #     points,
        #     pf.hermite.pf_fit_hermite_cubic,
        #     pf.SAMPLES_FAST,
        #     dt=self.DT, #self.getPeriod(),
        #     max_velocity=self.MAX_VELOCITY,
        #     max_acceleration=self.MAX_ACCELERATION,
        #     max_jerk=60
        # )

        # Convert the center trajectory to individual right and left side trajectories
        left, right = pf.modifiers.tank(trajectory, self.DRIVE_WIDTH)

        # Setup and configure the trajectory followers
        self.leftTrajectoryFollower = pf.followers.EncoderFollower(left)
        self.leftTrajectoryFollower.configureEncoder(
            self.leftMaster.getSelectedSensorPosition(self.kPIDLoopIdx), self.ENCODER_COUNTS_PER_REV, self.WHEEL_CIRCUMFERENCE
        )
        self.leftTrajectoryFollower.configurePIDSVA(0.0, 0.0, 0.0, self.KS, self.KV, self.KA)

        self.rightTrajectoryFollower = pf.followers.EncoderFollower(right)
        self.rightTrajectoryFollower.configureEncoder(
            -self.rightMaster.getSelectedSensorPosition(self.kPIDLoopIdx), self.ENCODER_COUNTS_PER_REV, self.WHEEL_CIRCUMFERENCE
        )
        self.rightTrajectoryFollower.configurePIDSVA(0.0, 0.0, 0.0, self.KS, self.KV, self.KA)

        start_time = self.timer.getFPGATimestamp()
        self.leftTrajectoryFollower.start(start_time)
        self.rightTrajectoryFollower.start(start_time)
    
    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""

        # Calculate the target x/y movement based on path finding
        now = self.timer.getFPGATimestamp()
        leftSpeed = self.leftTrajectoryFollower.calculate(self.leftMaster.getSelectedSensorPosition(self.kPIDLoopIdx), now)
        rightSpeed = self.rightTrajectoryFollower.calculate(self.rightMaster.getSelectedSensorPosition(self.kPIDLoopIdx), now)

        # Calculate target rotation using the gyro and pathfinding
        gyro_heading = -self.gyro.getAngle()
        if is_sim:
            gyro_heading = self.gyro_sim.getVoltage()
        desired_heading = pf.r2d(self.leftTrajectoryFollower.getHeading())
        
        angleDifference = pf.bound_angle(desired_heading - gyro_heading)
        turn = self.ANGLE_CONSTANT * angleDifference

        # Calculate target motor speeds (-1 to 1)
        leftSpeed = leftSpeed + turn
        rightSpeed = rightSpeed - turn

        self.leftMaster.setVoltage(leftSpeed)
        self.rightMaster.setVoltage(rightSpeed)

        if is_sim:
            self.left_motor_sim.set(-leftSpeed)
            self.right_motor_sim.set(rightSpeed)
        
        # Convert percent speed into encoder_ticks/100ms
        #leftVelocity = leftSpeed/(self.ENCODER_CONSTANT*10)
        #rightVelocity = rightSpeed/(self.ENCODER_CONSTANT*10)

        # Set motor speed
        #self.leftMaster.set(ControlMode.Velocity, leftVelocity)
        #self.rightMaster.set(ControlMode.Velocity, rightVelocity)

        
        # Display info for debugging
        #self.sd.putNumber("Right vel", self.rightMaster.getSelectedSensorVelocity(self.kPIDLoopIdx))
        #self.sd.putNumber("Target Right vel", rightVelocity)
        #self.sd.putNumber("Left vel", self.leftMaster.getSelectedSensorVelocity(self.kPIDLoopIdx))
        #self.sd.putNumber("Target Left vel", leftVelocity)

    def teleopInit(self):
        # Reset Gyro
        self.gyro.reset()

        # Store values for debugging
        self.maxSpeed = 0
        self.lastVelocity = 0
        self.maxAccel = 0

    def teleopPeriodic(self):
        # Get max speed
        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

        # Get turn and movement speeds
        yAxis = self.threshhold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed
        tAxis = self.threshhold(self.joystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed * (1 - abs(self.yAxis) * self.turnSlowdown) 

        # Emergency breaking
        if self.joystick.getRawButton(1):
            tAxis = 0
            yAxis = 0
        
        # Update motors
        self.drive.arcadeDrive(yAxis, tAxis)

        # Calculate and display info for debugging
        self.velocity = self.leftMaster.getSelectedSensorVelocity(self.kPIDLoopIdx) + self.rightMaster.getSelectedSensorVelocity(self.kPIDLoopIdx)
        self.velocity = self.velocity / 2
        if self.velocity > self.maxSpeed:
            self.maxSpeed = self.velocity
        self.lastVelocity = self.velocity
        
        self.sd.putNumber("Max Velocity", self.maxSpeed * self.ENCODER_CONSTANT * 10)
        self.sd.putNumber("Velocity", self.velocity * self.ENCODER_CONSTANT * 10)

        self.acceleration = (self.velocity - self.lastVelocity) / self.DT
        if self.acceleration > self.maxAccel:
            self.maxAccel = self.acceleration

        self.sd.putNumber("Acceleration", self.acceleration)
        self.sd.putNumber("Max Acceleration", self.maxAccel * self.ENCODER_CONSTANT * 10)

if __name__ == "__main__":
    wpilib.run(Robot)
