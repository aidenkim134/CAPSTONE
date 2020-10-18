import time
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from PID_control import PID
from motor_control import motor_control
import serial
from RPi.GPIO import GPIO

motor1 = motor_control([1,2,3,4])
motor1.setPins()

motor2 = motor_control([1,2,3,4])
motor2.setPins()


#variable for pid control
dist_ref = 0.05
dist_PID = PID(set_point=dist_ref)
dist_PID.setSampleTime(0.1)

pos_ref = 0.5
pos_PID = PID(set_point=pos_ref)
pos_PID.setSampleTime(0.1)

#serial connection
ser = serial.Serial("/dev/serial0", 115200)

usingPiCamera = True
# Set initial frame size.
frameSize = (320, 240)

# Initialize mutithreading the video stream.
vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=10).start()
# Allow the camera to warm up.
time.sleep(2.0)


def getTFminiData():
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)   
        ser.reset_input_buffer() 
        
        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            distance = distance / 100
            print("current distance is {}m".format(distance))
            ser.reset_input_buffer()
    return distance


        
        
        
        
class encoder:
    def __init__ (self, ins=[4,5]):
        self.rotation = 0
        self.aLastState = GPIO.input(ins[0])
        self.ins = ins
    
    def rotation (self):
        aState = GPIO.input(self.ins[0])
        
        if (aState != self.aLastState):
            if (GPIO.input(self.ins[1]) != aState):
                 self.rotation = 1
            else:
                self.rotation = -1
        else:
            self.rotation = 0
        self.aLastState = aState
        

RotL = encoder(ins=[20,21])
RotR = encoder(ins=[22,23])




#timeCheck = time.time()

Ballcolor = 0
Pt = [0,0,0]


def getPosition(Ldist, Rdist, Pt):
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


while True:
    time.sleep(0.1)
    
    distance = getTFminiData()
    # Get the next frame.
    frame = vs.read()
    # If using a webcam instead of the Pi Camera,
    # we take the extra step to change frame size.
    if not usingPiCamera:
        frame = imutils.resize(frame, width=frameSize[0])
 
    # Show video stream
    cv2.imshow('orig', frame)
    # if the `q` key was pressed, break from the loop.
    
    data = frame
    cameraPixels = sum(sum(data[:,:,Ballcolor] == 255))
    leftPixels = sum(sum(data[0:160,:, Ballcolor] == 255))
    rightPixels = sum(sum(data[160:320,:,Ballcolor] == 255))
    
    ratio = rightPixels / cameraPixels

    if cameraPixels < 500:
        motor1.setPWM(20); motor1.backward()
        motor2.setPWM(20); motor2.forward()
    
    elif (0.45 > ratio and ratio > 0.55):
        pos_PID.update(ratio)
        delta_pwm = pos_PID.output
        if ratio < 0.5:
            motor1.setPWM(delta_pwm); motor1.backward()
            motor2.setPWM(delta_pwm); motor2.forward()
        else:
            motor1.setPWM(delta_pwm); motor1.backward()
            motor2.setPWM(delta_pwm); motor2.forward()  
            
    else:
        
        pos_PID.update(ratio)
        delta_pwm = pos_PID.output
        if ratio > 0.5:
            delta_pwm = delta_pwm * -1
        
        dist_PID.update(distance)
        pwm = dist_PID.output
        motor1.setPWM(pwm-delta_pwm); motor1.forward()
        motor2.setPWM(pwm + delta_pwm); motor2.forward()
    
        if distance < 0.05:
            if Ballcolor == 0:
                Ballcolor = 1
            if Ballcolor == 1:
                Ballcolor = 0;
    
    Pt = getPosition(RotL.rotation, RotL.rotation, Pt)
        
    
    
#    print(1/(time.time() - timeCheck))
#    timeCheck = time.time()
    
# Cleanup before exit.
cv2.destroyAllWindows()
vs.stop()

