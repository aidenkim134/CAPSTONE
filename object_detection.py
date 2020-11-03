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
claw.open()

'''defining motor object'''
motor1 = motorControl([21, 20, 16])
motor2 = motorControl([1, 7 ,8])

'''variable for pid control'''
dist_ref = 0.1
dist_PID = PID(set_point=dist_ref)
dist_PID.setSampleTime(0.01)

# pos_ref = 0.5
# pos_PID = PID(set_point=pos_ref)
# pos_PID.setSampleTime(0.1)

'''serial connection for lidar'''
ser = serial.Serial("/dev/serial0", 115200)

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
            distance = 0.01
    return distance

def getPosition(Ldist, Rdist, Pt):
    '''obtain x, y, theta position based on encoder reading'''
    D = 15 #cm
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



'''define camera object to begin streaming'''
usingPiCamera = True
frameSize = (320, 240)

vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=10).start()
# Allow the camera to warm up.
time.sleep(2.0)

'''initial parameter for program starting'''
Ballcolor = 1
Pt = [0,0,0]



rot_speed = 20 #20% 
newBall = True


def UpdatePt (Pt):
    rotation1 = enc1.read()/ 10550; rotation2 = enc2.read() / 10550
    Pt = getPosition(rotation1, rotation2, Pt)
    return Pt

def GrabBall(Pt, ball, bound):
    Pt = Pt.copy()
    Pto = Pt
    
    while 95 < abs(Pt[2] - Pto[2]) < 85:
        time.sleep(0.01)
        motor1.setPWM(rot_speed); motor1.backward()
        motor2.setPWM(rot_speed); motor2.forward()
        rotation1 = enc1.read()/ 10550; rotation2 = enc2.read() / 10550
        Pt = getPosition(rotation1, rotation2, Pt)
        
    
    while np.sqrt(((Pt[0] - Pto[0])**2 - (Pt[1] - Pto[1])**2)) > 0.8:
        time.sleep
        
        motor1.setPWM(rot_speed); motor1.backward()
        motor2.setPWM(rot_speed); motor2.backward()
    
        rotation1 = enc1.read()/ 10550; rotation2 = enc2.read() / 10550
        Pt = getPosition(rotation1, rotation2, Pt)
    
    
    claw.close()
    if ball == 0:
        while (88 > Pt[2] or 92 < Pt[2]):
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt)
            
        while getTFminiData() > 0.1:
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.forward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt)
        
        while (358 > Pt[2] and 2 < Pt[2]):
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt) 
            
        while Pt[0] >bound[0]:
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.backward()
            Pt = UpdatePt (Pt) 
        
        claw.open()
            
    if ball == 1:
        while (268 > Pt[2] or 272 < Pt[2]):
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt)
        
        while getTFminiData() > 0.1:
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.forward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt)
    
        while (178 > Pt[2] or 182 < Pt[2]):
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
            Pt = UpdatePt (Pt) 
    
        while Pt[0] < bound[0]:
            time.sleep(0.01)
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.backward()
            Pt = UpdatePt (Pt) 
        claw.open()
    
enc_ref1 = 0; enc_ref2 = 0
IdentifyBound = False
bound = [[],[]]; edge = False;
try:
    while True:
        time.sleep(0.01)
        
        # Get the next frame.
        frame = vs.read()
        # If using a webcam instead of the Pi Camera,
        # we take the extra step to change frame size.
        if not usingPiCamera:
            frame = imutils.resize(frame, width=frameSize[0])
        
        # Show video stream
        cv2.imshow('orig', frame)
        key = cv2.waitKey(1) & 0xFF
     
        # if the `q` key was pressed, break from the loop.
        if key == ord("q"):
            break
        
        data = frame
        distance = getTFminiData()
        
        cameraPixels = sum(sum(data[:,:,Ballcolor] == 255))
        
        leftPixels = sum(sum(data[:,:160, Ballcolor] == 255))
        rightPixels = sum(sum(data[:,160: ,Ballcolor] == 255))
        ratio = rightPixels / (cameraPixels + 1)

        
        if IdentifyBound == False:
            if distance > 0.1 and edge == False:
                motor1.setPWM(rot_speed); motor1.forward()
                motor2.setPWM(rot_speed); motor2.forward()
                
            elif distance < 0.1 and  92 < Pt[2] > 88:
                edge = True
                motor1.setPWM(rot_speed); motor1.forward()
                motor2.setPWM(rot_speed); motor2.backward()
            else:
                bound[0].append(Pt[0])
                bound[1].append(Pt[1])
            if len(bound[0]) == 4:
                IdentifyBound = True


        elif cameraPixels < 300 :
            #rotate at 20 percent speed until it finds a ball
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
        

        elif ratio < 0.4:
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()
        elif ratio > 0.6:
            motor1.setPWM(rot_speed); motor1.backward()
            motor2.setPWM(rot_speed); motor2.forward()  
        
        elif (0.45 < ratio and ratio > 0.55):

            
            dist_PID.update(distance)
            pwm = dist_PID.output
            if pwm > 0:            
                motor1.setPWM(pwm); motor1.forward()
                motor2.setPWM(pwm); motor2.forward()
            
            if pwm < 0:
                motor1.setPWM(abs(pwm)); motor1.backward()
                motor2.setPWM(abs(pwm)); motor2.backward()
                
            if distance < 0.1:
                Pt = GrabBall(Pt, Ballcolor, bound)
                if Ballcolor == 0:
                    Ballcolor = 1
                if Ballcolor == 1:
                    Ballcolor = 0;

        rotation1 = enc1.read()/ 10550; rotation2 = enc2.read() / 10550
        Pt = getPosition(rotation1, rotation2, Pt)
        

except KeyboardInterrupt:
    motor1.stop();
    motor2.stop()
    cv2.destroyAllWindows()
    vs.stop()

