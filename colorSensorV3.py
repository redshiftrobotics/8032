from wpilib.i2c import I2C
from wpilib.driverstation import DriverStation
import hal
from hal_impl import i2c_helpers

class ColorSensorV3:
    """Custom driver for REV Color Sensor V3 over I2C"""
    def __init__(self, port):
        self.Register = {
            "kMainCtrl": 0,
            "kProximitySensorLED": 1,
            "kProximitySensorPulses": 2,
            "kProximitySensorRate": 3,
            "kLightSensorMeasurementRate": 4,
            "kLightSensorGain": 5,
            "kPartID": 6,
            "kMainStatus": 7,
            "kProximityData": 8,
            "kDataInfrared": 10,
            "kDataGreen": 13,
            "kDataBlue": 16,
            "kDataRed": 19
        }

        self.MainControl = {
            "kRGBMode": 4,
            "kLightSensorEnable": 2,
            "kProximitySensorEnable": 1,
            "OFF": 0
        }

        self.GainFactor = {
            "kGain1x": 0,
            "kGain3x": 1,
            "kGain6x": 2,
            "kGain9x": 3,
            "kGain18x": 4
        }

        self.LEDCurrent = {
            "kPulse2mA": 0,
            "kPulse5mA": 1,
            "kPulse10mA": 2,
            "kPulse25mA": 3,
            "kPulse50mA": 4,
            "kPulse75mA": 5,
            "kPulse100mA": 6,
            "kPulse125mA": 7
        }

        self.LEDPulseFrequency = {
            "kFreq60kHz": 24,
            "kFreq70kHz": 64,
            "kFreq80kHz": 40,
            "kFreq90kHz": 48,
            "kFreq100kHz": 56
        }

        self.ProximitySensorResolution = {
            "kProxRes8bit": 0,
            "kProxRes9bit": 8,
            "kProxRes10bit": 16,
            "kProxRes11bit": 24
        }

        self.ProximitySensorMeasurementRate = {
            "kProxRate6ms": 1,
            "kProxRate12ms": 2,
            "kProxRate25ms": 3,
            "kProxRate50ms": 4,
            "kProxRate100ms": 5,
            "kProxRate200ms": 6,
            "kProxRate400ms": 7
        }

        self.ColorSensorResolution = {
            "kColorSensorRes20bit": 0,
            "kColorSensorRes19bit": 8,
            "kColorSensorRes18bit": 16,
            "kColorSensorRes17bit": 24,
            "kColorSensorRes16bit": 32,
            "kColorSensorRes13bit": 40
        }

        self.ColorSensorMeasurementRate = {
            "kColorRate25ms": 0,
            "kColorRate50ms": 1,
            "kColorRate100ms": 2,
            "kColorRate200ms": 3,
            "kColorRate500ms": 4,
            "kColorRate1000ms": 5,
            "kColorRate2000ms": 7
        }
        self.kAddress = 82
        self.kPartID = -62
        simPort = None
        if hal.HALIsSimulation():
            simPort = ColorSensorV3Sim()
        self.m_i2c = I2C(port, self.kAddress, simPort=simPort)
        if (not self.checkDeviceID()):
            return None
        self.initializeDevice()
        self.hasReset()

    def configureProximitySensorLED(self, freq, curr, pulses):
        self.write8(self.Register["kProximitySensorLED"], freq | curr)
        self.write8(self.Register["kProximitySensorPulses"], pulses)

    def configureProximitySensor(self, res, rate):
        self.write8(self.Register["kProximitySensorRate"], res | rate)

    def configureColorSensor(self, res, rate, gain):
        self.write8(
            self.Register["kLightSensorMeasurementRate"], res | rate)
        self.write8(self.Register["kLightSensorGain"], gain)

    def getColor(self):
        r = self.getRed()
        g = self.getGreen()
        b = self.getBlue()
        mag = r + g + b
        return Color(r / mag, g / mag, b / mag)

    def getProximity(self):
        return self.read11BitRegister(self.Register["kProximityData"])

    def getRawColor(self):
        return RawColor(self.getRed(), self.getGreen(), self.getBlue(), self.getIR())

    def getRed(self):
        return self.read20BitRegister(self.Register["kDataRed"])

    def getGreen(self):
        return self.read20BitRegister(self.Register["kDataGreen"])

    def getBlue(self):
        return self.read20BitRegister(self.Register["kDataBlue"])

    def getIR(self):
        return self.read20BitRegister(self.Register["kDataInfrared"])

    def hasReset(self):
        raw = self.m_i2c.read(self.Register["kMainStatus"], 1)
        return ((raw & 0x20) != 0)

    def checkDeviceID(self):
        if (self.m_i2c.addressOnly()):
            DriverStation.reportError("Could not find REV color sensor", False)
            return False

        devId = self.m_i2c.read(self.Register["kPartID"], 1)

        if (devId != self.kPartID):
            DriverStation.reportError("Unknown device found with same I2C addres as REV color sensor", False)
            return False

        return True

    def initializeDevice(self):
        self.write8(self.Register["kMainCtrl"], self.MainControl["kRGBMode"] |
                    self.MainControl["kLightSensorEnable"] | self.MainControl["kProximitySensorEnable"])
        self.write8(self.Register["kProximitySensorRate"], self.ProximitySensorResolution["kProxRes11bit"]
                    | self.ProximitySensorMeasurementRate["kProxRate100ms"])
        self.write8(self.Register["kProximitySensorPulses"], 32)

    def read11BitRegister(self, reg):
        raw = self.m_i2c.read(reg, 2)
        return raw & 0x7FF

    def read20BitRegister(self, reg):
        raw = self.m_i2c.read(reg, 3)
        return raw & 0x3FFFF

    def write8(self, reg, data):
        self.m_i2c.write(reg, data)


class RawColor:
    def __init__(self, r, g, b, _ir):
        self.red = r
        self.green = g
        self.blue = b
        self.ir = _ir


class Color:
    def __init__(self, r, g, b):
        self.red = r
        self.green = g
        self.blue = b

class ColorSensorV3Sim(i2c_helpers.I2CSimBase):
    def readI2C(self, port, deviceAddress, buffer, count):
        return len(buffer)
    def transactionI2C(self, port, deviceAddress, dataToSend, sendSize, dataReceived, receiveSize):
        return len(dataReceived)