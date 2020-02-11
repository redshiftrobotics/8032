import wpilib
from wpilib.drive import DifferentialDrive

# Dictionary for identifying compatible inputs other than Logitech gamepad and joystick



class joystick_drive:
    
    # TODO: Make these enums?
    input_joystick_map = {
        "Saitek X52 Flight Control System": {
            "xChannel": 0,
            "yChannel": 1,
            "twistChannel": 5,
            "throttleChannel": 2
        },
        "Logitech Extreme 3D": {
            "xChannel": 0,
            "yChannel": 1,
            "twistChannel": 2,
            "throttleChannel": 3
        },
        "default": { # This matches the defaults as defined in wpilib.Joystick 
            "xChannel": 0,
            "yChannel": 1,
            "twistChannel": 3,
            "throttleChannel": 4
        }
    }
    
    def setHIDType(self, HIDType):
        """Mannually overrides the default HIDType from wpilib.interfaces.GenericHID"""
        self.HIDType = HIDType

    def getHIDType(self):
        """Returns the HIDType"""
        return self.HIDType

    def setJoystickName(self, joystickName):
        """Mannually overrides the default HIDName from wpilib.interfaces.GenericHID"""
        self.joystickName = joystickName

    def getJoystickName(self):
        """Returns the HIDName"""
        return self.joystickName

    def __init__(self, joystick_obj, speed, ySpeed, tSpeed):
        # Sets the speed
        self.speed = speed
        self.ySpeed = ySpeed
        self.tSpeed = tSpeed
        self.init_driver_control(joystick_obj)

    def init_driver_control(self, joystick_obj = wpilib.Joystick(0)):
        self.determine_control_scheme(joystick_obj)
        self.set_control_scheme(joystick_obj)


    def determine_control_scheme(self, joystick_obj):
        """Sets the values returned by wpilib.interfaces.GenericHID to self.HIDType and self.joystickName"""
        self.HIDType = joystick_obj.getType()
        self.joystickName = joystick_obj.getName()

    def set_control_scheme(self, joystick_obj):
        if self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick:
            joystick_obj.setXChannel(self.input_joystick_map[self.joystickName]["xChannel"])
            joystick_obj.setYChannel(self.input_joystick_map[self.joystickName]["yChannel"])
            joystick_obj.setTwistChannel(self.input_joystick_map[self.joystickName]["twistChannel"])
            joystick_obj.setThrottleChannel(self.input_joystick_map[self.joystickName]["throttleChannel"])

        #     self.calculateSpeeds = self.joystick_drive_teleop
        # else: # Defaults to tank drive with gamepad
        #     self.calculateSpeeds = self.gamepad_drive_teleop
    
    def threshold(self, value, limit):
         if (abs(value) < limit):
            return 0
         else:
            return round(value, 2)

    def calculateSpeeds(self, joystick_obj):
        if self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick:
            return self.joystick_drive_teleop(joystick_obj)
        else:
            return self.gamepad_drive_teleop(joystick_obj)

    def joystick_drive_teleop(self, joystick_obj):
        # Get max speed
        self.speed = (-joystick_obj.getThrottle() + 1)/2

        # Get turn and movement speeds
        tAxis = self.threshold(joystick_obj.getTwist(), 0.05) * self.tSpeed * self.speed
        yAxis = self.threshold(-joystick_obj.getY(), 0.05) * self.ySpeed * self.speed
        
        # Calculate right and left speeds
        leftSpeed = yAxis+tAxis
        rightSpeed = yAxis-tAxis

        return leftSpeed,rightSpeed

    def gamepad_drive_teleop(self, joystick_obj):
        # Get max speed
        
        self.tSpeed = self.joystick.getRawAxis(5)
        self.ySpeed = self.joystick.getRawAxis(5)

        self.ySpeed = self.ySpeed + 1
        self.tSpeed = self.tSpeed + 1
        self.ySpeed = self.ySpeed / 2
        self.tSpeed = self.tSpeed / 2

        self.tAxis = self.threshold(self.joystick.getRawAxis(3), 0.05) * self.tSpeed*-1
        self.yAxis = self.threshold(-self.joystick.getRawAxis(1), 0.05) * self.ySpeed 
         
        # figure out the state of the button 
        
        # Get turn and movement speeds
       
        # Calculate right and left speeds
        leftSpeed = self.yAxis
        rightSpeed = self.tAxis

        
        if self.button.get():
            leftSpeed = 0
            rightSpeed = 0
        
        
        # Update Motors
        self.frontLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.rearLeftTalon.set(ControlMode.PercentOutput, leftSpeed)
        self.frontRightTalon.set(ControlMode.PercentOutput, rightSpeed)
        self.rearRightTalon.set(ControlMode.PercentOutput, rightSpeed)

        # Update SmartDashboard
        self.sd.putNumber("Left Encoder", self.leftEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))
        self.sd.putNumber("Right Encoder", self.rightEncoder.getSelectedSensorPosition(self.kPIDLoopIdx))


        # TODO: insert gamepad drive here
        return 0,0

#class joystick_operator():
