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

config_file = "/boot/frc.json"

class CameraConfig: pass

team = None
table_name = "Main"
server = False
camera_configs = []
switched_camera_configs = []
cameras = []


# Copyright Levi Sprung 2020
def count_balls(img):
    """Counts the number of FRC power cells in an image"""
    shifted = cv2.pyrMeanShiftFiltering(img, 21, 51)
    blurred = cv2.GaussianBlur(shifted, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.erode(mask, None, iterations=20)
    mask = cv2.dilate(mask, None, iterations=20)


    D = ndimage.distance_transform_edt(mask)
    local_max = peak_local_max(D, indices=False, min_distance=20, labels=mask)

    markers = ndimage.label(local_max, structure = np.ones((3,3)))[0]
    labels = watershed(-D, markers, mask = mask)
    num_balls = len(np.unique(labels)) - 1

    return num_balls



def parse_error(str):
    """Report parse error."""
    print("config error in '" + config_file + "': " + str, file=sys.stderr)

def read_config():
    """Read configuration file."""
    global team
    global server

    # Parse file
    try:
        with open(config_file, "rt", encoding="utf-8") as f:
            config_json = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(config_file, err), file=sys.stderr)
        return False

    # Top level must be an object
    if not isinstance(config_json, dict):
        parse_error("must be JSON object")
        return False

    # Get team number
    try:
        team = config_json["team"]
    except KeyError:
        parse_error("could not read team number")
        return False

    # NetworkTables Mode (optional)
    if "ntmode" in config_json:
        str = config_json["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parse_error("could not understand ntmode value '{}'".format(str))

    return True

if __name__ == "__main__":

    # If there are arguments, the config is the first one
    if len(sys.argv) >= 2:
        config_file = sys.argv[1]

    # Read configuration
    if not read_config():
        sys.exit(1)

    # Start NetworkTables instance
    networktables_instance = NetworkTablesInstance.getDefault()

    # Server / Client switch
    if server:
        print("Setting up NetworkTables server")
        networktables_instance.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        networktables_instance.startClientTeam(team)
    
    # Create camera symbol
    usb_camera = cv2.VideoCapture(0)

    # Get the table to write data
    table = networktables_instance.getTable(table_name)
    balls_entry = table.getEntry("Balls")

    print("Logging to table", table_name)

    # While running...
    while True:
        ret, img = usb_camera.read()
        
        # If there is an error, wait then retry
        if ret == 0:
            time.sleep(0.1)
            continue

        balls = count_balls(img)
        
        # Update the networktable table
        balls_entry.setNumber(balls)