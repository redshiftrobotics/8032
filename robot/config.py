        #MISC
class MISC():
        kSlotIdx = 0
        kPIDLoopIdx = 0
        kTimeoutMs = 10
        # Sets the speed

class speed():
        speed = 1.0
        xSpeed = 1.0
        tSpeed = 2.0/3.0

class intake():
        intakeCollectToggleOn = False
        intakeReverse = 1
        intakeForward = 4
        baseIntakeSpeed = 0.4
        array = 7,0,1,0

class transit():
        transitAxis = 1 # [Left Joystick Y Axis]
        transitIndexSpeed = 0.5
        port = 6

        # Setup the hang
class hang():
        hangLeftAxis = 2
        hangRightAxis = 3
        hangExtend = 5
        
        leftMotor_ID = 8
        rightMotor_ID = 9
        maxSpeed = 0.3
        stopPoint = 0.3

        #Motors
class motors():
        leftMaster = 4 #port
        rightMaster = 5 
        leftAlt = 2
        rightAlt = 3
        kP = 0.085
        kD = 0
        kI = 0

        #Auto
class auto():

        # 3 Ball auto parameters
        depositTime = 0.75
        moveTime = 1.0
        #leftLeaveDist = 10_000 # encoder ticks
        #rightLeaveDist = 5_000# encoder ticks
        leaveTime = 0.7
        backwardsTime = 1.0
        leaveThreshold = 50
        # Initiation Line auto parameters
        initiationLineDistance = 2.0 # meters

        #Joystick
class joystick():
        driverJoystickLeftNumber = 0
        driverJoystickRightNumber = 1
        operatorJoystick = 2

        halfSpeedButton = 2 # [Thumb Left]
        stopButton = 1 # [Trigger]

        #Limelight
class limeLight():
        targetDistance = 15.0 # in
        targetBottomHeight = 89.5 # in
        # TODO: Tune limelight PID
        aimKp = 0.02
        distanceKp = -0.01
        minAimCommand = 0.05
        distanceThreshold = 20
        angleThreshold = 5
