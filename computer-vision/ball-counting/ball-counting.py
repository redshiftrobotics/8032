# imports necessary libraries
import cv2
import numpy as np
import imutils
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage
import time

vs = cv2.VideoCapture(0)

lower_yellow = np.array([20,75,75])
upper_yellow = np.array([35,255,255])

frames = 0


if vs.isOpened():
    ret, img = vs.read()
else:
    ret = False

while ret:
    start = time.time()

    ret, img = vs.read()
    shifted = cv2.pyrMeanShiftFiltering(img, 21, 51)
    blurred = cv2.GaussianBlur(shifted, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

    msk = cv2.inRange(hsv, lower_yellow, upper_yellow)

    msk = cv2.erode(msk,None,iterations = 20)
    msk = cv2.dilate(msk,None,iterations = 20)


    D = ndimage.distance_transform_edt(msk)
    localMax = peak_local_max(D, indices = False, min_distance = 20, labels = msk)

    markers = ndimage.label(localMax, structure = np.ones((3,3)))[0]
    labels = watershed(-D, markers, mask = msk)
    print("[INFO] {} unique segments found".format(len(np.unique(labels)) - 1))

    for label in np.unique(labels):
        if label == 0:
            continue
        
        mask = np.zeros(gray.shape, dtype="uint8")
        mask[labels == label] = 255
        ctrs = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ctrs = imutils.grab_contours(ctrs)
        c = max(ctrs, key=cv2.contourArea)
        contour = cv2.drawContours(img, ctrs, -1, (0, 255, 255), 3)
        (x, y), radius = cv2.minEnclosingCircle(c)
        center = (int(x), int(y))
        radius = int(radius)
        circle = cv2.circle(img, center, radius, (255, 255, 0), 2)
        
    cv2.imshow('r', img)

    k = cv2.waitKey(125)
        
    if k == 27:
        break

    current = time.time()
    fps = current - start

    print(fps)      

cv2.destroyWindow("preview")


    