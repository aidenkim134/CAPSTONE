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
        framerate=30).start()
# Allow the camera to warm up.
time.sleep(3.0)


def getSpeed (pre_enc1, pre_enc2, Time):
    
    vel1 = (enc1.read() - pre_enc1) / 36.5 / (time.time_ns() / 1E9 - Time) 
    vel2 = (enc2.read() - pre_enc2) / 36.5 / (time.time_ns() / 1E9 - Time)
    return vel1, vel2



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

def getPosition(Ldist, Rdist, Pt):
    '''obtain x, y, theta position based on encoder reading'''
    D = 0.156 #m
    if Ldist != Rdist:
        r = D * (Ldist + Rdist) / 2 / (Rdist - Ldist)
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
    rotation1 = vel1 * (2*np.pi) / 60 * (time.time_ns() / 1E9 - Time) *0.069 / 2 * 1.15 ; rotation2 = vel2 * (2*np.pi) / 60 *(time.time_ns() / 1E9 - Time) *0.069 / 2 * 1.13 

    Pt = getPosition(rotation1, rotation2, Pt)
    return Pt
p = []
def Camera(ballColor, began):
    if ballColor == 'red':
        colorLimit = [([0,0,80], [70, 20, 255])]
    if ballColor == 'blue':
        colorLimit = [([0,20,0], [255, 40, 50])]

    time.sleep(0.01)
    [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
    

    vs.camera.zoom = (0.45, 0.45, 0.45, 0.45)
    frame = vs.read()
    #only take red or blue color
    for (lower, upper) in colorLimit:
        lower = np.array(lower, dtype='uint8')
        upper = np.array(upper, dtype='uint8')
        
        mask = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        
    #find circular image using edge finding and hough transform (circle)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3.5, 80)

    if began == False:
        preCircle = np.array([[0,0,0]])
        began = True
    
    try:
        circles = np.round(circles[0, :]).astype(int)
        preCircle = circles.copy()
    except:
            circles = [[0,0,0]]

    for (x, y, r) in circles:
        cv2.circle(frame, (x,y), r, (0, 225, 0), 4)
        cv2.rectangle(frame, (x-2, y-2), (x+2, y+2), (0, 128, 225), -1)
        break
    p.append(Pt[2])
    # Show video stream
    cv2.imshow('orig', frame)
    key = cv2.waitKey(1) & 0XFF
    return x

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


rotation_speed= 30
forward_speed = 30
IdentifyBound = False

bound = [[],[]]; edge = False;

ballColor = 'red'
Pt = [0,0,0]

clearRef = 0
turnDeg = [90, 190, 280, 30]

try:
    while IdentifyBound == False:
        break
        Time = time.time_ns() / 1E9
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
        time.sleep(0.01)
        [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
        distance = getTFminiData()
        if distance == 1E9:
            Stop(motor1, motor2)
            w_PID1.clear(); w_PID2.clear()
            continue

            
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
            edge = False
                    
            bound[0].append(Pt[0])
            bound[1].append(Pt[1])
                    
            if len(bound[0]) == 4:
                IdentifyBound = True
                motor1.stop()
                motor2.stop()
        Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
        

    while True:
        '''Locating the ball'''
        rotationAng = np.linspace(5, 100,19)
        delay = 0
        
        began = False
        direction = 'clockwise'
        Reference = 0
        FoundBall = False
        
        while True:
            clearRef = 0
            d = []
            
            while FoundBall == False:
                Exit = False
                distance = getTFminiData()
                if distance < 0.2:
                    Stop(motor1, motor2)
                    Exit = True
                    break
                if distance == 1E9:
                    Stop(motor1, motor2)
                    clearRef = 0
                
                Time = time.time_ns() / 1E9
                pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
                time.sleep(0.01)
                

                [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
                x = Camera(ballColor, began)
 
                if (Pt[2] > rotationAng[-1] or Reference == 18) and Pt[2] < 300:
                    direction ='counter clockwise'
                    Reference = 18        
                elif Pt[2] < rotationAng[0] or Pt[2] > 300 or Reference == 0:
                    direction ='clockwise'
                    Reference = 0
            
                if (70 > round(x, -1) or round(x, -1) > 270):
                    if clearRef == 0:
                        w_PID1.clear(); w_PID2.clear()
                        clearRef = 1
                    if direction == 'clockwise':
                        if  (rotationAng[int(Reference)] > Pt[2] or Pt[2] > 300):
                            Rotate(15, vel1, vel2)
                        else:
                            Stop(motor1, motor2)
                            Reference = Reference + 1 / 2
                            clearRef = 0
                            
                    if direction == 'counter clockwise':
                        if  (rotationAng[int(Reference)] < Pt[2]) and Pt[2] < 300:
                            RotateCC(15, vel1, vel2)
                        else:
                            Stop(motor1, motor2)
                            
                            Reference = Reference - 1 / 2
                            clearRef = 0
                        
                else:
                    FoundBall = True
                    Ptb = Pt
            
                Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
                p.append(Pt[2])
            

            
            if Exit == True:
                Stop(motor1, motor2)
                break
            clearRef = 0
            x = 0
            while FoundBall == True:
                Exit = False
                distance = getTFminiData()
                if distance < 0.2:
                    Stop(motor1, motor2)
                    Exit = True
                    break
                if distance == 1E9:
                    Stop(motor1, motor2)
                    clearRef = 0
                Time = time.time_ns() / 1E9
                pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
                time.sleep(0.01)
                
                [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
                d.append(distance)
                
                if clearRef == 0:
                    w_PID1.clear(); w_PID2.clear()
                    clearRef = 1
                        

                x_prime = Camera(ballColor, began)
                if x_prime == 0:
                    delay = delay + 1
                else:
                    delay = 0
                    x = x_prime  
                if delay > 6:
                    delay = 0
                    x = x_prime
                    FoundBall = False

                speed = 8
                Forward(speed, vel1, vel2)
   
                Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
                p.append(Pt[2])

            if Exit == True:
                Stop(motor1, motor2)
                break

        '''Once the object reaches the ball'''

        #first rotate  and position to align with the claw
        Pto = [0,0,0]
        clear = True
        while 60 != round(Pto[2]-5, -1):
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
                
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            RotateCC(5, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
            Pto = UpdatePt(Pto, pre_enc1, pre_enc2, Time)
            p.append(Pt[2])   
        Stop(motor1, motor2)
        time.sleep(1)

        #Move backward slightly to enclose the ball with claw
        clear = True
        while np.sqrt((Pto[0])**2 + (Pto[1])**2) < distance * 130:
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Backward(5, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
            Pto = UpdatePt(Pto, pre_enc1, pre_enc2, Time)
            p.append(Pt[2])
        Stop(motor1, motor2)
        time.sleep(1)
        #close the claw to get a hold of the ball
        claw.clawClose()
        
        '''Transporting the ball to the respective locations'''
        
        #Find the edge perpendicular the location of inventory
        if ballColor == 'red':
            wall = 240; gate = 350; #pos = bound[0][3]-20#pos = bound[3][0]
        if ballColor == 'blue':
            wall = 70; gate = 180; #pos = bound[1][1]
        
        clear = True
        while wall != round(Pt[2], -1):
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Rotate(20, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
            p.append(Pt[2]) 
        Stop(motor1, motor2)
     
        
        
        #run through the lidar few times to prevent erroneous data
        i = 0
        while i < 10:
            time.sleep(0.1)
            distance = getTFminiData()
            i = i + 1
 
        
        #move forward until reaching the edge perpendicular to inventory
        clear = True
        while distance > 0.2:
        
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                w_PID1.resetGain(); w_PID2.resetGain()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Forward(forward_speed, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
            distance = getTFminiData()
            if distance == 0:
                distance = 1E9
        Stop(motor1, motor2)  

        #Turn to face away from the inventory
        clear = True
        while gate != round(Pt[2], -1):
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Rotate(rotation_speed, vel1, vel2)
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
        Stop(motor1, motor2)
        time.sleep(1)

        #move to the inventory
        while i < 10:
            time.sleep(0.1)
            distance = getTFminiData()
            i = i + 1
        distance = 0
        
        clear = True
        while distance < 1.1:
            distance = getTFminiData()
            if distance == 1E9:
                Stop(motor1, motor2)
                w_PID1.clear(); w_PID2.clear()
                distance = 0
                    
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Backward(7, vel1, vel2)
        
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
        Stop(motor1, motor2)
        claw.clawOpen()
        
        
        
        while i < 10:
            time.sleep(0.1)
            distance = getTFminiData()
            i = i + 1
        distance = 0
        
        clear = True
        while distance > 0.95:
            distance = getTFminiData()
            if distance == 1E9:
                Stop(motor1, motor2)
                w_PID1.clear(); w_PID2.clear()
                
            Time = time.time_ns() / 1E9
            pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
            time.sleep(0.01)
            if clear:
                w_PID1.clear(); w_PID2.clear()
                clear = False
            [vel1, vel2] = getSpeed(pre_enc1, pre_enc2, Time)
            Forward(7, vel1, vel2)
        
            Pt = UpdatePt(Pt, pre_enc1, pre_enc2, Time)
        Stop(motor1, motor2)
        Pt[2] = 0    
        #if ballColor == "blue":
            #break
        #open the claw to release the ball
        if ballColor == 'red':
            ballColor = 'blue';
        elif ballColor == 'blue':
            ballColor = 'red';
            
except KeyboardInterrupt:
    Stop(motor1, motor2)
    cv2.destroyAllWindows()
    vs.stop()











    



