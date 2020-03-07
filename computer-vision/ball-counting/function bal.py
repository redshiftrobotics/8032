# Import general libraries
import json
import sys
import time

# Import communication libraries

# Import CV libraries
import cv2
import numpy as np
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage

lower_yellow = np.array([20,75,75])
upper_yellow = np.array([35,255,255])

camera_configs = []
switched_camera_configs = []
cameras = []


# Copyright Levi Sprung 2020
def count_balls(img):
    """Counts the number of FRC power cells in an image"""
    # shifted = cv2.pyrMeanShiftFiltering(img, 21, 51)
    # blurred = cv2.GaussianBlur(shifted, (11, 11), 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.dilate(mask, None, iterations=8)
    mask = cv2.erode(mask, None, iterations=25)
    mask = cv2.dilate(mask, None, iterations=10)

    cv2.imshow("mask", mask)


    D = ndimage.distance_transform_edt(mask)
    local_max = peak_local_max(D, indices=False, min_distance=20, labels=mask)

    markers = ndimage.label(local_max, structure = np.ones((3,3)))[0]
    labels = watershed(-D, markers, mask = mask)
    num_balls = len(np.unique(labels)) - 1
    
    print(num_balls)
    
    cv2.imshow("img", img)
    return num_balls
    

usb_camera = cv2.VideoCapture(1)

# While running...
while True:
    ret, img = usb_camera.read()
    
    # If there is an error, wait then retry
    if ret == 0:
        time.sleep(0.1)
        continue

    balls = count_balls(img)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()