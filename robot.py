#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from state import State
from state import DRIVE_FORWARD_TWO_SEC, FREEZE, TANK_DRIVE_NORMAL

from ctre import WPI_TalonSRX, NeutralMode

# ENCODER_COUNT = 2

class Robot(wpilib.TimedRobot):
    # ENCODER_PULSE_PER_REV = 1024
    # WHEEL_DIAMETER = 0.5
    # 
    # talon_id_set = [
    #     ('Front Left', 5, False),
    #     ('Front Right', 1, False),
    #     ('Back Left', 4, False),
    #     ('Back Right', 15, False)
    # ]
    # 
    # encoder_value_sets = []  # tuples of (home_pos, min_pos, max_pos)
    # last_encoder_values = []
    # talon_sweeping = []
    
    def threshhold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)

    def robotInit(self):
        #: Which PID slot to pull gains from. Starting 2018, you can choose from
        #: 0,1,2 or 3. Only the first two (0,1) are visible in web-based
        #: configuration.
        self.kSlotIdx = 0

        #: Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
        #: now we just want the primary one.
        self.kPIDLoopIdx = 0

        #: set to zero to skip waiting for confirmation, set to nonzero to wait and
        #: report to DS if action fails.
        self.kTimeoutMs = 10
        
		# Sets the speed
        self.speed = 0.4
        self.ySpeed = 1
        self.tSpeed = 0.75
        
        self.yAxis = 0
        self.tAxis = 0

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # joysticks 1 on the driver station
        self.joystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()
        
        self._state = State(self, FREEZE)
        
        self.frontLeftTalon = WPI_TalonSRX(2)
        self.rearLeftTalon = WPI_TalonSRX(0)
        self.frontRightTalon = WPI_TalonSRX(3)
        self.rearRightTalon = WPI_TalonSRX(1)

        self.frontLeftTalon.setNeutralMode(NeutralMode.Brake)
        self.rearLeftTalon.setNeutralMode(NeutralMode.Brake)
        self.frontRightTalon.setNeutralMode(NeutralMode.Brake)
        self.rearRightTalon.setNeutralMode(NeutralMode.Brake)

        self.frontLeftTalon.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.rearLeftTalon.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.frontRightTalon.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.rearRightTalon.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
            self.kPIDLoopIdx,
            self.kTimeoutMs,
        )

        self.leftEncoder = self.rearLeftTalon
        self.rightEncoder = self.rearRightTalon

    @property
    def state(self):
        return self._state.state
        
    @state.setter
    def state(self, new_state):
        self._state.dispatch(new_state)
        
    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        self._state.update()

        #if self.state == DRIVE_FORWARD_TWO_SEC:
        #    self.myRobot.tankDrive(0.3, 0.3)
        #elif self.state == FREEZE:
        #    self.myRobot.tankDrive(0, 0)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):      
        self._state.update()

        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

        self.tAxis = self.threshhold(self.joystick.getRawAxis(2), 0.05) * self.tSpeed * self.speed
        self.yAxis = self.threshhold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed * self.speed
        
        self.frontLeftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, self.yAxis+self.tAxis)
        self.rearLeftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, self.yAxis+self.tAxis)
        self.frontRightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, self.yAxis-self.tAxis)
        self.rearRightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, self.yAxis-self.tAxis)

        self.sd.putNumber("Left Encoder", self.leftEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))
        self.sd.putNumber("Right Encoder", self.rightEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))

if __name__ == "__main__":
    wpilib.run(Robot)
