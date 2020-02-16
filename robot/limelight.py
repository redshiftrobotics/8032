#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables
import math

LIMELIGHT_LED_ON = 3
LIMELIGHT_LED_OFF = 1

MOUNT_ANGLE = 43.711  # deg
MOUNT_HEIGHT = 23.375  # 23 5/16 in
LIMELIGHT_HORIZONTAL_FOV = 24.85 * 2 # deg


class LimeLight:
    """Custom driver for limelight over NetworkTables"""

    def __init__(self):
        self.table = NetworkTables.getTable("limelight")

    # The distance to a target height
    def getDistance(self, targetHeight):
        """:returns Returns the distance to a given object at a specific height"""

        # If there is a valid target
        if self.getTV() > 0.0:
            # The vertical angle
            targetAngle = self.getTY()

            # Determine difference in height
            deltaHeight = targetHeight - MOUNT_HEIGHT

            # Get the angle to the ground from the target
            totalAngle = MOUNT_ANGLE + targetAngle

            # height / tan(angle) = length (distance)
            return deltaHeight / math.tan(math.radians(totalAngle))

        else:
            # Must check in parent function if getDistance is None
            return None

    # Whether the limelight has any valid targets (0 or 1)
    def getTV(self):
        """:returns Whether the limelight has any valid targets"""
        return self.table.getNumber('tv', 0.0)

    # The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    def getLatency(self):
        """:returns The pipeline’s latency contribution"""
        return self.table.getNumber('tl', 0.0)

    # Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    def getTX(self):
        """:returns Horizontal Offset From Crosshair To Target"""
        return self.table.getNumber('tx', 0.0)

    # Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    def getTY(self):
        """:returns Vertical Offset From Crosshair To Target"""
        return self.table.getNumber('ty', 0.0)

    # Target Area (0% of image to 100% of image)
    def getTA(self):
        """:returns Target Area"""
        return self.table.getNumber('ta', 0.0)

    # Skew or rotation (-90 degrees to 0 degrees)
    def getTS(self):
        """:returns Skew or rotation"""
        return self.table.getNumber('ts', 0.0)

    # Sets the LED state
    def setLedMode(self, value):
        """Sets the LED state"""
        self.table.putNumber('ledMode', value)

    def toggleLed(self):
        """Toggles the LED on or off"""
        current = self.table.getNumber('ledMode', 0.0)

        if current != 1:
            self.setLedMode(1)
        else:
            self.setLedMode(3)

    # Sets limelight’s operation mode
    def setCamMode(self, value):
        """Sets the camera mode"""
        self.table.putNumber('camMode', value)
