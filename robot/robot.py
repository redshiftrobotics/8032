#!/usr/bin/env python3
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from state import State
from state import DRIVE_FORWARD_TWO_SEC, FREEZE, TANK_DRIVE_NORMAL
import cv2
import numpy as np
from ctre import WPI_TalonSRX, ControlMode, NeutralMode, FeedbackDevice

class Robot(wpilib.TimedRobot):
    def threshold(self, value, limit):
         if (abs(value) < limit):
             return 0
         else:
             return round(value, 2)

    def robotInit(self):
		# Sets the speed
        self.speed = 0.5
        
        self.yAxis = 0
        self.tAxis = 0

        # Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # The [a] button debounced
        self.button = ButtonDebouncer(wpilib.Joystick(0), 0)
        
        # Motors for driving
        self.frontLeftMotor = wpilib.Talon(2)
        self.rearLeftMotor = wpilib.Talon(0)
        self.frontRightMotor = wpilib.Talon(3)
        self.rearRightMotor = wpilib.Talon(1)

        # Motor controller groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # Robot drivetrain
        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        
        # Create a simple timer (docs: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Timer.html#wpilib.timer.Timer.get)
        self.timer = wpilib.Timer()
        
        self._state = State(self, FREEZE)

        # gets the image
        self.vs = cv2.VideoCapture(0)

        """
        FOCAL LENGTH = 564.6 px
        """

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

        # distance between ball and camera
    def getDistance(self, width, focalLength, pixels):
        return width * focalLength / pixels

    # creates one function that shows the image and closes after a key is pressed
    def showImg(self, txt, image):
        cv2.imshow(txt, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def autonomousPeriodic(self):
    
        self._state.update()

        if self.state == DRIVE_FORWARD_TWO_SEC:
            self.myRobot.tankDrive(0.3, 0.3)
        elif self.state == FREEZE:
            self.myRobot.tankDrive(0, 0)

        if self.vs.isOpened():
            ret, img = self.vs.read()
        else:
            ret = False

        while ret:
            # reads and displays each frame
            ret, img = self.vs.read()

            # converts to hsv
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

            # creates yellow color boundaries
            lower_yellow = np.array([20,75,50])
            upper_yellow = np.array([40,255,255])

            # creates mask to isolate the ball within the image
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # erodes and dilates pixels to get rid of small spots
            mask = cv2.erode(mask,None,iterations = 10)
            mask = cv2.dilate(mask,None,iterations = 10)

            # gets the contours
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # converts mask image to rgb
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

            # draws the contours
            # ctrs = cv2.drawContours(img, contours, -1, (255, 0, 255), 3)

            # gets a contour
            for ctr in contours:

                # creates a circle that encloses the contour
                (x,y), radius = cv2.minEnclosingCircle(ctr)
                center = (int(x), int(y))
                radius = int(radius)
                circle = cv2.circle(img, center, radius, (255, 0, 255), 2)


        # focal length dimensions
                distance1 = 19 # inches, distance from camera
                if radius:
                    pixels1 = 208 # pixels, diameter of ball
                else:
                    radius = 1
                    pixels1 = 0
                width1 = 7 # inches, diameter of ball
                focalLength1 = pixels1 * distance1 / width1 
                
                distanceFromCamera = self.getDistance(width1, focalLength1, radius*2)
                distanceFromCamera = round(distanceFromCamera, 2)
                cv2.putText(img, f"{distanceFromCamera} in", center, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                self.logger.info(f"Distance from ball: {distanceFromCamera}")
        
        

                # shows the image
            # cv2.imshow('a', img)
            k = cv2.waitKey(125)
            if k == 27:
                break

        cv2.destroyWindow("preview")


    def teleopInit(self):
        self.myRobot.setSafetyEnabled(True)

        # Tests setting the debounce period
        self.button.set_debounce_period(0.8)

    def teleopPeriodic(self):
        # Get max speed
        self.speed = (-self.joystick.getRawAxis(3) + 1)/2

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
