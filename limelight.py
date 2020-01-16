#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables

class LimeLight:
    """Custom driver for limelight over NetworkTables"""
    def __init__(self):
        self.table = NetworkTables.getTable("limelight")

    # Whether the limelight has any valid targets (0 or 1)
    def getTV(self):
        """:returns Whether the limelight has any valid targets"""
        return self.table.getNumber('tv', None)

    # The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    def getLatency(self):
        """:returns The pipeline’s latency contribution"""
        return self.table.getNumber('tl', None)
    
    # Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    def getTX(self):
        """:returns Horizontal Offset From Crosshair To Target"""
        return self.table.getNumber('tx', None)

    # Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    def getTY(self):
        """:returns Vertical Offset From Crosshair To Target"""
        return self.table.getNumber('ty', None)
        
    # Target Area (0% of image to 100% of image)
    def getTA(self):
        """:returns Target Area"""
        return self.table.getNumber('ta', None)

    # Skew or rotation (-90 degrees to 0 degrees)
    def getTS(self):
        """:returns Skew or rotation"""
        return self.table.getNumber('ts', None)

    # Sets the LED state
    def setLedMode(self, value):
        """Sets the LED state"""
        self.table.putNumber('ledMode', value)

    # Sets limelight’s operation mode
        """Sets the camera mode"""
        self.table.putNumber('camMode', value)

