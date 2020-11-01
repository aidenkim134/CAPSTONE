import time
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from PID_control import PID
from motor_control import motorControl
import serial
import RPi.GPIO as GPIO

'''defining motor object'''
motor1 = motorControl([21, 20, 16, 13, 19])
motor2 = motorControl([1, 7 ,8 ,5, 6])

'''variable for pid control'''
dist_ref = 0.05
dist_PID = PID(set_point=dist_ref)
dist_PID.setSampleTime(0.1)

pos_ref = 0.5
pos_PID = PID(set_point=pos_ref)
pos_PID.setSampleTime(0.1)

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
    return Pt



'''define camera object to begin streaming'''
usingPiCamera = True
frameSize = (320, 240)

vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=10).start()
# Allow the camera to warm up.
time.sleep(2.0)

'''initial parameter for program starting'''
Ballcolor = 0
Pt = [0,0,0]




try:
    while True:
        time.sleep(0.1)
        
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
        
        motor1.setPWM(100); motor1.forward()
        
        if cameraPixels < 300 :
            motor1.setPWM(100); motor1.forward()
            motor2.setPWM(100); motor2.forward()
        
        elif (0.2 > ratio and ratio > 0.8):
            pos_PID.update(ratio-0.5)
            delta_pwm = pos_PID.output
            if ratio < 0.5:
                motor1.setPWM(delta_pwm); motor1.backward()
                motor2.setPWM(delta_pwm); motor2.forward()
            else:
                motor1.setPWM(delta_pwm); motor1.backward()
                motor2.setPWM(delta_pwm); motor2.forward()  
            print('from second: delta_pwm = {}'.format(delta_pwm))     
        else:
            pos_PID.update(ratio-0.5)
            delta_pwm = pos_PID.output
            
            dist_PID.update(distance-0.05)
            pwm = dist_PID.output 
            motor1.setPWM(pwm-delta_pwm); motor1.forward()
            motor2.setPWM(pwm + delta_pwm); motor2.forward()
            print('from third: delta_pwm = {}'.format(delta_pwm))
            print('from third: pwm = {}'.format(pwm))
            if distance < 0.05:
                if Ballcolor == 0:
                    Ballcolor = 1
                if Ballcolor == 1:
                    Ballcolor = 0;
        motor1.countRotation(); motor2.countRotation()
        Pt = getPosition(motor1.rotation, motor2.rotation, Pt)
        print(motor1.pwm)
        print (ratio)
except KeyboardInterrupt:
    motor1.setPWM(0); motor1.forward();
    motor2.setPWM(0); motor2.forward()
    cv2.destroyAllWindows()
    vs.stop()

