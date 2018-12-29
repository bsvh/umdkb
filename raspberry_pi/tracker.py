#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
import time

# run with -v option to see webcam video for calibration

# values to change
camera = 0 # ID of webcam (usually 0 or 1)
x_range = (720,750) # x limits of scanning window
y_range = (125,510) # y limits of scanning window

time_start = 0
cap = cv2.VideoCapture(camera)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);

first_run = True

while (cap.isOpened()):
    if not first_run:
        print(time.clock()-time_start)
    time_start = time.clock()
    ret, frame = cap.read()
    bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img = bw[y_range[0]:y_range[1],x_range[0]:x_range[1]]
    lines = np.average(img, axis=1)
    cutoff = np.amin(lines) + 5
    
    if first_run:
        plt.plot(range(*y_range),lines)
        plt.axhline(cutoff)
        plt.savefig('lines.png')
        first_run = False
    _, w = bw.shape

    position = np.where(lines < cutoff)[0][0] + y_range[0] # this value is the vertical position in pixels

    frame[position,:,:] = [[0,0,255]]*w
    if len(sys.argv)>1 and sys.argv[1] == '-v':
        frame[y_range[0],x_range[0]:x_range[1],:] = [0,0,255]
        frame[y_range[1],x_range[0]:x_range[1],:] = [0,0,255]
        frame[y_range[0]:y_range[1],x_range[0],:] = [0,0,255]
        frame[y_range[0]:y_range[1],x_range[1],:] = [0,0,255]
        cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # code needs to be written to do something with the position

cap.release()
