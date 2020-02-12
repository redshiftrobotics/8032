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
        print (self.joystickName)

    def set_control_scheme(self, joystick_obj):
        if self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick and self.joystickName in self.input_joystick_map:
            joystick_obj.setXChannel(self.input_joystick_map[self.joystickName]["xChannel"])
            joystick_obj.setYChannel(self.input_joystick_map[self.joystickName]["yChannel"])
            joystick_obj.setTwistChannel(self.input_joystick_map[self.joystickName]["twistChannel"])
            joystick_obj.setThrottleChannel(self.input_joystick_map[self.joystickName]["throttleChannel"])
        elif self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick and not(self.joystickName in self.input_joystick_map):
            print("Unknown joystick name used on port " + str(joystick_obj.getPort()) + ": " + self.joystickName + "; will use default mapping. Please use a different controller or setup a new mapping.")
        #     self.calculateSpeeds = self.joystick_drive_teleop
        # else: # Defaults to tank drive with gamepad
        #     self.calculateSpeeds = self.gamepad_drive_teleop
        # print("Unknown controller name used detected on port " + str(joystick_obj.getPort()) + ": " + self.joystickName + "; will use default mapping. Please use a different controller or setup a new mapping.")
    
    def threshold(self, value, limit): # TODO: Values are not continuous, e.g. as soon as the value is above the limit, the return value jumps from 0 to the limit 
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
        # Get max speed from throttle
        self.speed = (-joystick_obj.getThrottle() + 1)/2

        # Get turn and movement speeds
        tAxis = self.threshold(joystick_obj.getTwist(), 0.05) * self.tSpeed * self.speed
        yAxis = self.threshold(-joystick_obj.getY(), 0.05) * self.ySpeed * self.speed
        
        # Calculate right and left speeds
        leftSpeed = yAxis+tAxis
        rightSpeed = yAxis-tAxis

        return leftSpeed,rightSpeed

    def gamepad_drive_teleop(self, joystick_obj):
        # TODO: insert gamepad drive here
        return 0,0

#class joystick_operator():

class joystick_operator:
    
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
        "Xbox Controller": { # This matches the defaults as defined in wpilib.Joystick 
            "xChannel": 0,
            "yChannel": 1,
            "twistChannel": 3,
            "throttleChannel": 4,
            "cellTransportFWD":4 ,
            "cellTransportBWD": 5,
            "intakeMotor":-1 ,
            "winchUp":-1 ,
            "winchDown": -1
            
            
            

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
        if self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick and self.joystickName in self.input_joystick_map:
            joystick_obj.setXChannel(self.input_joystick_map[self.joystickName]["xChannel"])
            joystick_obj.setYChannel(self.input_joystick_map[self.joystickName]["yChannel"])
            joystick_obj.setTwistChannel(self.input_joystick_map[self.joystickName]["twistChannel"])
            joystick_obj.setThrottleChannel(self.input_joystick_map[self.joystickName]["throttleChannel"])
        elif self.HIDType == wpilib.interfaces.GenericHID.HIDType.kHIDJoystick and not(self.joystickName in self.input_joystick_map):
            print("Unknown joystick name used on port " + str(joystick_obj.getPort()) + ": " + self.joystickName + "; will use default mapping. Please use a different controller or setup a new mapping.")
        #     self.calculateSpeeds = self.joystick_drive_teleop
        # else: # Defaults to tank drive with gamepad
        #     self.calculateSpeeds = self.gamepad_drive_teleop
        # print("Unknown controller name used detected on port " + str(joystick_obj.getPort()) + ": " + self.joystickName + "; will use default mapping. Please use a different controller or setup a new mapping.")
    
    def threshold(self, value, limit): # TODO: Values are not continuous, e.g. as soon as the value is above the limit, the return value jumps from 0 to the limit 
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
        # Get max speed from throttle
        self.speed = (-joystick_obj.getThrottle() + 1)/2

        # Get turn and movement speeds
        tAxis = self.threshold(joystick_obj.getTwist(), 0.05) * self.tSpeed * self.speed
        yAxis = self.threshold(-joystick_obj.getY(), 0.05) * self.ySpeed * self.speed
        
        # Calculate right and left speeds
        leftSpeed = yAxis+tAxis
        rightSpeed = yAxis-tAxis

        return leftSpeed,rightSpeed

    def gamepad_drive_teleop(self, joystick_obj):
        # TODO: insert gamepad drive here
        return 0,0
