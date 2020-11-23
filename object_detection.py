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

'''defining encoder object'''
enc1 = Encoder.Encoder(13,19)
enc2 = Encoder.Encoder(5, 6)

'''definining stepper motor claw object'''
stepper = [27, 22, 23, 24] #in1, in2, in3, in4
claw = clawControl(stepper)


'''defining motor object'''
motor1 = motorControl([21, 16, 12])
motor2 = motorControl([1, 7 ,8])

'''defining PID object'''
w_PID1 = PID(set_point = 0)
w_PID2 = PID(set_point = 0)




'''serial connection for lidar'''
ser = serial.Serial("/dev/serial0", 115200)


'''define camera object to begin streaming'''
usingPiCamera = True
frameSize = (320, 240)

vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=20).start()
# Allow the camera to warm up.
time.sleep(3.0)


rotation_speed= 30
forward_speed = 30
Ptb = 1000

def getSpeed (pre_enc1, pre_enc2, Time):
    
    vel1 = (enc1.read() - pre_enc1) / 36.5 / (time.time_ns() / 1E9 - Time) 
    vel2 = (enc2.read() - pre_enc2) / 36.5 / (time.time_ns() / 1E9 - Time)
    return vel1, vel2



def getTFminiData():
    while True:
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
            else:
                distance = 0
                return distance  

def getPosition(Ldist, Rdist, Pt):
    '''obtain x, y, theta position based on encoder reading'''
    D = 0.188 #m
    if Ldist != Rdist:
        r = D * (Ldist + Rdist) / 2 / np.abs(Rdist - Ldist)
        Pt[2] = (Rdist - Ldist) / D * 360 / (2*np.pi) + Pt[2]
        Pt[0] = r * np.cos(Pt[2] *2*np.pi / 360) + Pt[0]
        Pt[1] = r * np.sin(Pt[2]*2*np.pi / 360) + Pt[1]
    else:
        Pt[0] = Ldist * np.cos(Pt[2]*2*np.pi / 360) + Pt[0] 
        Pt[1] = Ldist * np.sin(Pt[2]*2*np.pi / 360) + Pt[1]
        Pt[2] = Pt[2] 
    
    Pt[2] = Pt[2] % 360
    return Pt

def UpdatePt (Pt, pre_enc1, pre_enc2, Time):
    [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
    #print ('actual speed is: {}, {}'.format(vel1, vel2))
    rotation1 = vel1 * (2*np.pi) / 60 * (time.time_ns() / 1E9 - Time) *0.069 / 2 * 1.15 ; rotation2 = vel2 * (2*np.pi) / 60 *(time.time_ns() / 1E9 - Time) *0.069 / 2 * 1.125 

    Pt = getPosition(rotation1, rotation2, Pt)
    return Pt


    


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
    
def Stop (motor1, motor2):
    motor1.setPWM(0); motor1.forward()
    motor2.setPWM(0); motor2.forward()

def GrabBall(Pt, ballColor, bound):
    Pt = Pt.copy()
    Pto = Pt
    clear = True
    while 190 < abs(Pt[2] - Pto[2]) or 170 > abs(Pt[2] - Pto[2]):
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
        time.sleep(0.1)
        if clear:
            w_PID1.clear(); w_PID2.clear()
            clear = False
        [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
        Rotate(rotation_speed, vel1, vel2)
        
        Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
    Stop(motor1, motor2)
    
    clear = True
    while np.sqrt((Pt[0] - Pto[0])**2 + (Pt[1] - Pto[1])**2) < 0.05:
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
        time.sleep(0.1)
        if clear:
            w_PID1.clear(); w_PID2.clear()
            clear = False
        [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
        Backward(forward_speed, vel1, vel2)
        
        Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
    Stop(motor1, motor2)
    
    claw.close()
    
    if ballColor == 'red':
        clear = True
        while (85  > Pt[2] or 95 < Pt[2]):
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.1)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
            Rotate(rotation_speed, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
        Stop(motor1, motor2)
        
        clear = True
        while getTFminiData() > 0.2:
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.1)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
            Forward(forward_speed, vel1, vel2)
        
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
        Stop(motor1, motor2)  
        
        clear = True
        while (175 < Pt[2]):
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.1)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
            Rotate(rotation_speed, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
        Stop(motor1, motor2)
        clear = True
        while Pt[0] < bound[0]:
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.1)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
            Backward(forward_speed, vel1, vel2)
            
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2)
        Stop(motor1, motor2)
        
        claw.open()
            
    if ballColor == 'blue':
        while (268 > Pt[2] or 272 < Pt[2]):
            time.sleep(0.1)
            Rotate(motor1, motor2, 40)
            Pt = UpdatePt (Pt)
        Stop(motor1, motor2)
        
        while getTFminiData() > 0.1:
            time.sleep(0.1)
            Forward(motor1, motor2, 80)
            Pt = UpdatePt (Pt)
        Stop(motor1, motor2)
        
        while (182 < Pt[2]):
            time.sleep(0.1)
            Rotate(motor1, motor2, 40)
            Pt = UpdatePt (Pt) 
        Stop(motor1, motor2)
        
        while Pt[0] < bound[0]:
            time.sleep(0.1)
            Backward(motor1, motor2, 80)
            Pt = UpdatePt (Pt) 
        Stop(motor1, motor2)
        
        claw.open()
    



#coordinates of the bound and whether they have been identified or not 
IdentifyBound = False
bound = [[],[]]; edge = False;

ballColor = 'red'

#minimum distance for the robot to approach object
minDist = 0.1
began = False
'''initial parameter for program starting'''
Pt = [0,0,0]
clearRef = 2
theta = []
wait_time = 0 #ms
direction = 0
FoundBall = False
try:
    while True:
        Time = time.time_ns() / 1E9
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
        if ballColor == 'red':
            colorLimit = [([0,0,50], [70, 30, 255])]
        if ballColor == 'blue':
            colorLimit = [([0,0,0], [50, 40, 200])]
            
        time.sleep(0.01)
        [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)

        # Get the next frame.
        vs.camera.zoom = (0.45, 0.45, 0.45, 0.45)
        frame = vs.read()
        view = frame.copy()
        #only take red or blue color
        for (lower, upper) in colorLimit:
            lower = np.array(lower, dtype='uint8')
            upper = np.array(upper, dtype='uint8')
            
            mask = cv2.inRange(frame, lower, upper)
            frame = cv2.bitwise_and(frame, frame, mask=mask)
            
        #find circular image using edge finding and hough transform (circle)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 4.5, 80)

        if began == False:
            preCircle = np.array([[0,0,0]])
            began = True
        
        try:
            circles = np.round(circles[0, :]).astype(int)
            preCircle = circles.copy()
        except:
            circles = [[0, 0, 0]]

            
        for (x, y, r) in circles:
            cv2.circle(frame, (x,y), r, (0, 225, 0), 4)
            cv2.rectangle(frame, (x-2, y-2), (x+2, y+2), (0, 128, 225), -1)
            break
        
        # Show video stream
        cv2.imshow('orig', frame)
        key = cv2.waitKey(1) & 0xFF
     
        # if the `q` key was pressed, break from the loop.
        if key == ord("q"):
            break

        
        distance = getTFminiData()
        if distance == 0:
            distance = 1E9
        Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
    
        turnDeg = [90, 180, 270, 0]

        #if  IdentifyBound == False:
        
        if False:
            
            if distance > 0.2:
                speed = forward_speed
            if distance <= 0.2:
                speed = 0

            if distance > 0.2 and edge==False:
                if clearRef == 0:
                    w_PID1.clear(); w_PID2.clear()
                    clearRef = 1
               
                Forward(speed, vel1, vel2)

                
            elif  (turnDeg [len(bound[0])] !=  round(Pt[2]-5, -1)%360):

                if clearRef == 1:
                    w_PID1.clear(); w_PID2.clear()
                    clearRef = 0

                edge = True
                Rotate(rotation_speed, vel1, vel2)
                
            else:

                print(Pt)

                
                edge = False
                
                bound[0].append(Pt[0])
                bound[1].append(Pt[1])
                
            if len(bound[0]) == 4:
                IdentifyBound = True
                motor1.stop()
                motor2.stop()
                clearRef = 2
                break
        elif (20 > x or x > 300):
            if clearRef == 2:
                w_PID1.clear(); w_PID2.clear()
                clearRef = 3
            
            elif  (0 !=  round(Pt[2]-5, -1)%20):
                if FoundBall == True:
                    if Ptb < Pt[2]:
                        RotateCC(15, vel1, vel2)
                    else:
                        Rotate(15, vel1, vel2)
                else:
                    Rotate(15, vel1, vel2)
                i = 0
            elif i < 100:
                Forward(0, vel1, vel2)
                i = i + 1
        
        elif (20 < x < 300):
            if clearRef == 3:
                w_PID1.clear(); w_PID2.clear()
                clearRef = 2

            Ptb = Pt[2]; FoundBall = True
            if distance > 0.2:
                speed = forward_speed
            elif distance <= 0.2:
                speed = 0
            
            Forward(speed, vel1, vel2)
                
            if distance < 0.2:
                break
                Pt = GrabBall(Pt, ballColor, bound)
                if ballColor == 'red':
                    ballColor = 'blue';
                if ballColor == 'blue':
                    ballColor = 'red';
                
        theta.append(Pt[2])
                
    plt.figure(1)
    plt.plot(theta)
    plt.show()
    

except KeyboardInterrupt:
    motor1.stop();
    motor2.stop();
    cv2.destroyAllWindows()
    vs.stop()

