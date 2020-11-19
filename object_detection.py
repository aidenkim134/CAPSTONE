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

'''defining encoder object'''
enc1 = Encoder.Encoder(13,19)
enc2 = Encoder.Encoder(5, 6)

'''definining stepper motor claw object'''
stepper = [27, 22, 23, 24] #in1, in2, in3, in4
claw = clawControl(stepper)


'''defining motor object'''
motor1 = motorControl([21, 16, 20])
motor2 = motorControl([1, 7 ,8])

'''defining PID object'''
w_PID1 = PID(set_point = 40)
w_PID2 = PID(set_point = 40)



'''serial connection for lidar'''
ser = serial.Serial("/dev/serial0", 115200)


'''define camera object to begin streaming'''
usingPiCamera = True
frameSize = (320, 240)

vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=10).start()
# Allow the camera to warm up.
time.sleep(2.0)



def getSpeed (pre_enc1, pre_enc2):
    
    vel1 = (enc1.read() - pre_enc1) / 10550 / 0.1 * 2 * np.pi
    vel2 = (enc2.read() - pre_enc2) / 10550 / 0.1 * 2 * np.pi
    print(vel1)
    return vel1, vel2



def getTFminiData():
    '''obtain distance value reading from lidar based on serial connection'''
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)   
        ser.reset_input_buffer() 
        
        if recv[0] == 0x59 and recv[1] == 0x59:
            distance = recv[2] + recv[3] * 256
            distance = distance / 100
            print("current distance is {}m".format(distance))
            ser.reset_input_buffer()
        else:
            distance = 0
    return distance

def getPosition(Ldist, Rdist, Pt):
    '''obtain x, y, theta position based on encoder reading'''
    D = 5 #cm
    if Ldist != Rdist:
        r = D * (Ldist + Rdist) / 2 / (Rdist - Ldist)
        Pt[2] = (Rdist - Ldist) / D + Pt[2]
        Pt[0] = r * np.cos(Pt[2]) + Pt[0]
        Pt[1] = r * np.sin(Pt[2]) + Pt[1]
    else:
        Pt[0] = Ldist * np.cos(Pt[2]) + Pt[0] 
        Pt[1] = Ldist * np.sin(Pt[2]) + Pt[1]
        Pt[2] = Pt[2]
    
    Pt[2] = Pt[2] % 360
    return Pt

def UpdatePt (Pt):
    rotation1 = enc1.read()/ 10550; rotation2 = enc2.read() / 10550
    Pt = getPosition(rotation1, rotation2, Pt)
    return Pt

def Rotate(motor1, motor2, speed):
    w_PID.update(speed);
    pwm = w_PID.output
    motor1.setPWM(pwm); motor1.backward()
    motor2.setPWM(pwm); motor2.forward()
    
def RotateCC(motor1, motor2,speed):
    w_PID.update(speed);
    pwm = w_PID.output
    motor1.setPWM(pwm); motor1.forward()
    motor2.setPWM(pwm); motor2.backward()

def Backward(motor1, motor2, speed):
    w_PID.update(speed);
    pwm = w_PID.output
    motor1.setPWM(pwm); motor1.backward()
    motor2.setPWM(pwm); motor2.backward()

def Forward(motor1, motor2, speed, vel1, vel2):

    w_PID1.update(vel1) ; w_PID1.update(vel2)
    print(w_PID1.output)
    motor1.setPWM(w_PID1.output); motor1.forward()
    motor2.setPWM(w_PID2.output); motor2.forward()

def Stop (motor1, motor2):
    motor1.setPWM(0); motor1.forward()
    motor2.setPWM(0); motor2.forward()

def GrabBall(Pt, ballColor, bound):
    Pt = Pt.copy()
    Pto = Pt

    while 185 < abs(Pt[2] - Pto[2]) or abs(Pt[2] - Pto[2]) < 175:
        time.sleep(0.1)
        Rotate(motor1, motor2, 40)
        Pt = UpdatePt(Pt)
    Stop(motor1, motor2)
    
    while abs(Pt[0] - Pto[0]) > 0.05 and abs(Pt[1] - Pto[1]) > 0.05:
        time.sleep(0.1)
        Backward(motor1, motor2, 50)
        Pt = UpdatePt(Pt)
    Stop(motor1, motor2)
    
    claw.close()
    
    if ballColor == 'red':
        while (88 > Pt[2] or 92 < Pt[2]):
            time.sleep(0.1)
            Rotate(motor1, motor2, 40)
            Pt = UpdatePt (Pt)
        Stop(motor1, motor2)  
        while getTFminiData() > 0.1:
            time.sleep(0.1)
            Forward(motor1, motor2, 50)
            Pt = UpdatePt (Pt)
        Stop(motor1, motor2)   
        
        while (2 < Pt[2]):
            time.sleep(0.1)
            Rotate(motor1, motor2, 40)     
            Pt = UpdatePt (Pt) 
        Stop(motor1, motor2)
        
        while Pt[0] > bound[0]:
            time.sleep(0.1)
            Backward(motor1, motor2, 80)
            Pt = UpdatePt (Pt) 
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

try:
    while True:
        pre_enc1 = enc1.read(); pre_enc2 = enc2.read()
        if ballColor == 'red':
            colorLimit = [([0,0,80], [225, 70, 225])]
        if ballColor == 'blue':
            colorLimit = [([30,0,0], [225, 60, 100])]
        
        time.sleep(0.1)
        [vel1, vel2] = getSpeed(pre_enc1, pre_enc2)
        Forward(motor1, motor2, 40, vel1, vel2)
        Pt = UpdatePt(Pt)
        print(Pt)
        continue
        # Get the next frame.
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
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2.5, 50)
        if began == False:
            preCircle = np.array([[0,0,0]])
            began = True
        
        try:
            circles = np.round(circles[0, :]).astype(int)
            preCircle = circles.copy()
        except:
            circles = preCircle
            
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
        continue
        
        data = frame
        distance = getTFminiData()

        
        turnDeg = [270, 180, 90, 0]
        if IdentifyBound == False:
            if distance > 0.1:
                speed = 80
            if distance < 0.1:
                speed = 0

            
            if distance > 0.1 and edge==False:
                Forward(motor1, motor2, speed)
            
            elif turnDeg [len(bound[0])] == int(Pt[2]):
                edge = True
                Rotate(motor1, motor2, 40)
                
            else:
                edge = False
                
                bound[0].append(Pt[0])
                bound[1].append(Pt[1])
            
            if len(bound[0]) == 4:
                IdentifyBound = True

        elif 165 < x < 155:
            speed = 40
            #rotate at 20 percent speed until it finds a ball
            Rotate(motor1, motor2, speed)

        elif (165 > x > 155):
            if distance > 0.1:
                speed = 80
            if distance < 0.1:
                speed = 0
            
            Forward(motor1, motor2, speed)
                
            if distance < 0.1:
                Pt = GrabBall(Pt, ballColor, bound)
                if ballColor == 'red':
                    ballColor = 'blue'
                if ballColor == 'blue':
                    ballColor = 'red';

        Pt = UpdatePt(Pt)
        

except KeyboardInterrupt:
    motor1.stop();
    motor2.stop()
    cv2.destroyAllWindows()
    vs.stop()

