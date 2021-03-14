# -*- coding: utf-8 -*-
"""
Created on Sun Feb 28 15:56:00 2021

@author: KIMAIDE
"""
import time
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from PID_control import PID
from motor_control import motorControl
import serial
import RPi.GPIO as GPIO
from claw_control import clawControl
import Encoder
import matplotlib.pyplot as plt
from LandmarkFinding import FindLandmark
from PathPlanning import PathPlanning


'''defining encoder object'''
enc1 = Encoder.Encoder(13,19); enc2 = Encoder.Encoder(2, 3)

'''defining motor object'''
motor1 = motorControl([21, 16, 12])
motor2 = motorControl([1, 7 ,8])

'''definining stepper motor claw object'''
stepper = [27, 22, 23, 24] #in1, in2, in3, in4
claw = clawControl(stepper)


'''defining PID object'''
w_PID1 = PID(set_point = 0)
w_PID2 = PID(set_point = 0)



def getSpeed (d_enc1, d_enc2, dt):
    
    Ldist = (d_enc1) / 36.5 * (2 * np.pi) / 60 * 0.069 / 2 * 1.15 
    Rdist = (d_enc2) / 36.5 * (2 * np.pi) / 60 * 0.069 / 2 * 1.15 
    

    D = 0.156 #m
    if Ldist != Rdist:
        r = D * (Ldist + Rdist) / 2 / (Rdist - Ldist)
        Theta = (Rdist - Ldist) / D * 360 / (2*np.pi) 

    else:
        r = Ldist
        Theta = 0
    
    Theta = Theta % 360

    vel1 = (enc1.read() - pre_enc1) / 36.5 / (dt) 
    vel2 = (enc2.read() - pre_enc2) / 36.5 / (dt)
    
    V = r / dt 
    W = Theta / dt
    
    return vel1, vel2, V, W

def Rotate(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.backward()
    motor2.setPWM(w_PID2.output); motor2.forward()
    
def RotateCC(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.backward()

def Backward(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.backward()
    motor2.setPWM(w_PID2.output); motor2.backward()

def Forward(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    #print('pwm output is: {}, {}'.format(w_PID1.output, w_PID2.output))
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.forward()
    

    
SLAM = FindLandmark()
FoundBall = False
Time = time.time_ns() / 1E9
pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
motion = 'rotate'
clearTerms = False




while True:
    
    while FoundBall == False:

        
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.update(V, W, dt)
    
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   
    
    
        if Time % 2 == 0:
            SLAM.store()
        
        
        dx = SLAM.slam.u0[SLAM.slam.ClosestIDX] - SLAM.slam.u0[0]
        dy = SLAM.slam.u0[SLAM.slam.ClosestIDX + 1] - SLAM.slam.u0[1]
        
        Theta_rel = SLAM.slam.u0[2] - np.arctan2(dy / dx)
        min_dist = np.sqrt(dx**2 + dy**2)
        
        
        adjuster = 360
        if Theta_rel > 0:
            adjuster = -360
    
        if abs(Theta_rel) > 180:
            Theta_rel = adjuster + Theta_rel
        
        if abs(Theta_rel) > 15:
            if motion == 'forward':
                clearTerms = True
            motion = 'rotate'
            
        else:
            if motion == 'rotate':
                clearTerms = True
            motion = 'forward'
            
        if motion == 'rotate':
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear();
                clearTerms = False
            if Theta_rel > 0:
                Rotate (10, vel1, vel2)
            if Theta_rel < 0:
                RotateCC (10, vel1, vel2)
                
        elif motion =='forward':
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear(); 
                clearTerms = False
            Forward (30, vel1, vel2)
            
        
        if min_dist < 0.1:
            w_PID1.clear(); w_PID2.clear(); 
            Forward(0, vel1, vel2)
            
            
            # Ballcolor = getColor()
            
            foundBall = True
    
    while foundBall == True:
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.update(V, W, dt)
    
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   
    
    
        if Time % 2 == 0:
            SLAM.store()
            
            
        dx = SLAM.slam.u0[SLAM.slam.ClosestIDX] - SLAM.slam.u0[0]
        dy = SLAM.slam.u0[SLAM.slam.ClosestIDX + 1] - SLAM.slam.u0[1]
        
        Theta_rel = SLAM.slam.u0[2] - np.arctan2(dy / dx)
        min_dist = np.sqrt(dx**2 + dy**2)
        
        adjuster = 360
        if Theta_rel > 0:
            adjuster = -360
    
        if abs(Theta_rel) > 180:
            Theta_rel = adjuster + Theta_rel
            
        
        if abs(Theta_rel) < 180:
            if motion == 'backward':
                clearTerms = True
            motion = 'rotate'
        elif min_dist > 0.1:
            if motion == 'rotate':
                clearTerms = True
            motion = 'backward'
        else:
            w_PID1.clear(); w_PID2.clear(); 
            Forward(0, vel1, vel2)
            claw.Close()
            
            Time = time.time_ns() / 1E9  
            FoundBall = False
        
        
        if motion == 'rotate':
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear(); 
                clearTerms = False
            Rotate(10, vel1, vel2)
            
            
            w_PID1.clear(); w_PID2.clear(); 
        elif motion == 'backward':
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear(); 
                clearTerms = False
            Backward(15, vel1, vel2)
            
    while True:
        path = PathPlanning(SLAM.slam.u0, 'max?')
        points = path.getPoints()
        
        
#every nth iteration, do 5 scans.

# move towards landmark

# locate to a ball and run CNN. Grab it if certain covariance is reached

# Calculate path and Bring to landmark with highest x and y or lowest x and  depending on CNN value

#rotate drop and repeat the process.