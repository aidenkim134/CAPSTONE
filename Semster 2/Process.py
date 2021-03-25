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
enc1 = Encoder.Encoder(2,3); enc2 = Encoder.Encoder(13, 19)

'''defining motor object'''
motor1 = motorControl([1, 8, 7])
motor2 = motorControl([21, 16, 12])

'''definining stepper motor claw object'''
stepper = [27, 22, 23, 24] #in1, in2, in3, in4
claw = clawControl(stepper)


'''defining PID object'''
w_PID1 = PID(set_point = 0)
w_PID2 = PID(set_point = 0)



def getSpeed (d_enc1, d_enc2, dt):
    
    Ldist = (d_enc1)  / 6500
    Rdist = (d_enc2)  / 6500
    

    D = 0.205 #m
    if Ldist != Rdist:
        r = D * (Ldist + Rdist) / 2 / (Rdist - Ldist)
        Theta = (Rdist - Ldist) / D * 360 / (2*np.pi) 

    else:
        r = np.inf
        Theta = 0

    vel1 = (d_enc1) / 6500 / (dt) 
    vel2 = (d_enc2) / 6500 / (dt)
    
    W = Theta * np.pi / 180 / dt
    
    if W == 0:
        V = Ldist / dt
    elif W != 0:
        V = W * r
    
    return vel1, vel2, V, W

def RotateCC(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.backward()
    motor2.setPWM(w_PID2.output); motor2.forward()
    
def Rotate(speed, vel1, vel2):
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
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.forward()
    

    






SLAM = FindLandmark()
# do two scan before beginning


while True:
    
    for j in range(2):
        plt.figure()
        SLAM.Store()
        plt.plot(SLAM.slam.u0[0], SLAM.slam.u0[1], 'd')
        for i in range(3, len(SLAM.slam.u0), 2):
            plt.plot(SLAM.slam.u0[i], SLAM.slam.u0[i+1], 'x')
        plt.savefig('pics/landmarks{}.png'.format(j))

    foundBall = False
    Time = time.time_ns() / 1E9
    pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
    motion = 'rotate'
    clearTerms = False
    Destination = False
    break
    while foundBall == False:

        
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2;
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.Update(V, W, dt)
        
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   
            

        
        dx = SLAM.slam.u0[SLAM.slam.ClosestIdx] - SLAM.slam.u0[0]
        dy = SLAM.slam.u0[SLAM.slam.ClosestIdx + 1] - SLAM.slam.u0[1]
        
        Theta_rel = SLAM.slam.u0[2] - np.arctan2(dy, dx) * 180 / np.pi
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
                print('clear terms')
            if Theta_rel > 0:
           
                Rotate (3, vel1, vel2)
    
            if Theta_rel < 0:
           
                RotateCC (3, vel1, vel2)
         
        elif motion =='forward':
            
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear(); 
                clearTerms = False
            Forward (3, vel1, vel2)
            
        
        if min_dist < 0.05:
            SLAM.slam.u0[SLAM.slam.ClosestIdx + 1] = 0
            SLAM.slam.u0[SLAM.slam.ClosestIdx] = 0

            w_PID1.clear(); w_PID2.clear(); 
            Forward(0, vel1, vel2)
            ballColor = 'red'
            
            # ballColor = getColor()        
            if ballColor == 'red':
                coord = 'max'
            elif ballColor == 'blue':
                coord = 'min'
            
            print('We found the ball-------------')

            path = PathPlanning(SLAM.slam.u0, coord)

            points = path.getPoints()
            print(points)
            foundBall = True
    
    while foundBall == True:
      
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.Update(V, W, dt)
    
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   

        dx = SLAM.slam.u0[SLAM.slam.ClosestIdx] - SLAM.slam.u0[0]
        dy = SLAM.slam.u0[SLAM.slam.ClosestIdx + 1] - SLAM.slam.u0[1]
        
        Theta_rel = SLAM.slam.u0[2] - np.arctan2(dy, dx)
        min_dist = np.sqrt(dx**2 + dy**2)
        
        adjuster = 360
        if Theta_rel > 0:
            adjuster = -360
    
        if abs(Theta_rel) > 180:
            Theta_rel = adjuster + Theta_rel
            
        
        if abs(Theta_rel) > 175:
            w_PID1.clear(); w_PID2.clear(); 
            Forward(0, vel1, vel2)
            claw.Close()
            
            Time = time.time_ns() / 1E9  
            Destination = True
            foundBall = False
            print('we got the ball')
        

        Rotate(3, vel1, vel2)
            
    idx = 0
    while Destination == True:
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2;
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.Update(V, W, dt)
        
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   
            
        dx = points[idx][0] - SLAM.slam.u0[0]
        dy = points[idx][1]- SLAM.slam.u0[1]
        
        Theta_rel = SLAM.slam.u0[2] - np.arctan2(dy, dx) * 180 / np.pi
        min_dist = np.sqrt(dx**2 + dy**2)
        
        adjuster = 360
        if Theta_rel > 0:
            adjuster = -360
    
        if abs(Theta_rel) > 180:
            Theta_rel = adjuster + Theta_rel
        
        if abs(Theta_rel) > 5:
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
                print('clear terms')
            if Theta_rel > 0:
     
                Rotate (3, vel1, vel2)
    
            if Theta_rel < 0:
            
                RotateCC (3, vel1, vel2)
         
        elif motion =='forward':
            
            if clearTerms == True:
                w_PID1.clear(); w_PID2.clear(); 
                clearTerms = False
            Forward (3, vel1, vel2)
            
        
        if min_dist < 0.1:
            if len(points) == 2 and idx == 0:
                idx = 1
                continue
            else:
                w_PID1.clear(); w_PID2.clear(); 
                Forward(0, vel1, vel2)
                Destination = False

    Terminate = False
    while Terminate == False:
        time.sleep(0.01)
        dt = time.time_ns() / 1E9 - Time
        d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
        
        vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
        SLAM.slam.Update(V, W, dt)
    
    
        Time = time.time_ns() / 1E9    
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()   
      
        RotateCC(3, vel1, vel2)
  
            
        if SLAM.slam.u0[2] > 180 and coord == 'max' or SLAM.slam.u0[2] < 5 and coord == 'min':
            w_PID1.clear(); w_PID2.clear(); 
            Forward(0, vel1, vel2)
            claw.Open()
            Terminate = True
            
            SLAM.slam.u0[3:] = 0
        
        
    
        
        
    #for point in points:
        # rotate
        
        # approach

    
    #once approached turn 180 ish:
    #claw.close()
    
        
        # loop through two points
        
#every nth iteration, do 5 scans.

# move towards landmark

# locate to a ball and run CNN. Grab it if certain covariance is reached

# Calculate path and Bring to landmark with highest x and y or lowest x and  depending on CNN value

#rotate drop and repeat the process.