# Import general libraries
import json
import sys
import time

# Import communication libraries
from networktables import NetworkTablesInstance
import ntcore

# Import CV libraries
import cv2
import numpy as np
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage

lower_yellow = np.array([20,75,75])
upper_yellow = np.array([35,255,255])

def count_balls(img):
    shifted = cv2.pyrMeanShiftFiltering(img, 21, 51)
    blurred = cv2.GaussianBlur(shifted, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    msk = cv2.inRange(hsv, lower_yellow, upper_yellow)

    msk = cv2.erode(msk, None, iterations=20)
    msk = cv2.dilate(msk, None, iterations=20)


    D = ndimage.distance_transform_edt(msk)
    localMax = peak_local_max(D, indices=False, min_distance=20, labels=msk)

    markers = ndimage.label(localMax, structure = np.ones((3,3)))[0]
    labels = watershed(-D, markers, mask = msk)
    num_balls = len(np.unique(labels)) - 1

    return num_balls

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    return True

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
    
    usbCamera = cv2.VideoCapture(0)

    # loop forever
    while True:
        ret, img = usbCamera.read()
        
        if ret == 0:
            time.sleep(0.4)
            continue
        
        balls = count_balls(img)
        print(balls)
