# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 08:09:52 2021

@author: KIMAIDE
"""

import numpy as np
from numpy import matmul as mul
from servo import ServoControl
import serial
import EKFSLAM
from sklearn.cluster import DBSCAN
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets

servo = ServoControl(26)

ser = serial.Serial("/dev/serial0", 115200)
def getTFminiData():
    i = 1
    while i < 500:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)   
            ser.reset_input_buffer() 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                distance = distance / 100
                #print("current distance is {}m".format(distance))
                ser.reset_input_buffer()
                return distance  
        i = i + 1
    return 1E9        

slam = EKFSLAM()

Ranges = []; Bearings = []
for theta in range(-45, 45, 5):
    servo.TurnTo(theta)
    
    distance = getTFminiData()
    Ranges.append(distance)
    
    Bearings.append((slam.u0[2] + theta)%360)

servo.TurnTo(0)
print(Ranges, Bearings)




      