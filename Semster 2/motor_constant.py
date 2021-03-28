from motor_control import motorControl
import RPi.GPIO as GPIO 
import time
import Encoder
import pandas as pd
from PID_control import PID
import numpy as np
def Forward(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.forward()
    
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


def getSpeed (d_enc1, d_enc2, dt):
    #10000 was good tick ratio
    Ldist = (d_enc1)  / 6500
    Rdist = (d_enc2)  / 6500
    

    D = 0.26 #m
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

'''defining PID object'''
w_PID1 = PID(set_point = 0)
w_PID2 = PID(set_point = 0)

motor1 = motorControl([1, 8, 7])

enc1 = Encoder.Encoder(2,3)

motor2= motorControl([21, 16, 12])

enc2= Encoder.Encoder(13,19)

time.sleep(2)



startTime = time.time_ns() / 1E9
pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
i = 0
d = []; theta = []
while i< 500:
    time.sleep(0.01)
    dt = time.time_ns() / 1E9 - startTime
    
    startTime = time.time_ns() / 1E9
    d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
    
    vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)
    print(V, W)
    pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
    
    Rotate(3, vel1, vel2)
    
    d.append(V * dt); theta.append(W * dt * 180 / np.pi)
    print(enc1.read(), enc2.read())
 
    i = i + 1
motor1.setPWM(0); motor1.stop()
motor2.setPWM(0); motor2.stop()
print(sum(d))
print(sum(theta)%360)
#data = pd.DataFrame({'time':data[0], 'value':data[1]})
#data.to_csv('data.csv')