from motor_control import motorControl
import RPi.GPIO as GPIO 
import time
import Encoder
import pandas as pd

def Forward(speed, vel1, vel2):
    w_PID1.SetPoint = speed; w_PID2.SetPoint = speed; 
    w_PID1.update(vel1) ; w_PID2.update(vel2)
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.forward()

def getSpeed (d_enc1, d_enc2, dt):
    
    Ldist = (d_enc1)  * (2 * np.pi) / 60 * 0.069 / 2 
    Rdist = (d_enc2)  * (2 * np.pi) / 60 * 0.069 / 2  
    

    D = 0.205 #m
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

'''defining PID object'''
w_PID1 = PID(set_point = 0)
w_PID2 = PID(set_point = 0)

motor2 = motorControl([1, 8, 7])

enc2 = Encoder.Encoder(2,3)

motor1 = motorControl([21, 16, 12])

enc1 = Encoder.Encoder(13,19)

time.sleep(2)



startTime = time.time_ns() / 1E9
pre_enc1 = enc1.read(); pre_enc2 = enc2.read()

while True:
    time.sleep(0.01)
    dt = time.time_ns() / 1E9 - startTime
    d_enc1 = enc1.read() - pre_enc1; d_enc2 = enc2.read() - pre_enc2; 
    
    vel1, vel2, V, W = getSpeed(d_enc1, d_enc2, dt)

    pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
    
    Forward(20, vel1, vel2)



motor1.setPWM(0); motor1.stop()
#data = pd.DataFrame({'time':data[0], 'value':data[1]})
#data.to_csv('data.csv')