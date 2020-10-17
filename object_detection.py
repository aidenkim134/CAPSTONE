import time
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from PID_control import PID
from motor_control import motor_control
import serial

motor1 = motor_control([1,2,3,4])
motor1.setPins()

motor2 = motor_control([1,2,3,4])
motor2.setPins()



#variable for pid control
dist_ref = 0.05
dist_PID = PID(set_point=dist_ref)
dist_PID.setSampleTime(0.1)

pos_PID = PID(set_point=dist_ref)
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


counter = 0 # use this to measure our motor positions
aLastState = GPIO.input(31)
while True:
    astate = GPIO.input(31)
    if (aState != aLastState):
        if (GPIO.input(29) != aState):
            counter = counter + 1
        else
            counter = counter - 1
        print('the position of the motor = {}/n rad'.format(counter))
        
    aLastState = aState


#timeCheck = time.time()
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
    key = cv2.waitKey(1) & 0xFF
 
    # if the `q` key was pressed, break from the loop.
    if key == ord("q"):
        break
    
    data = frame
    
    process = data[125:175, 125:175]
    value = sum(sum(data[:,:,0] == 255))
    print(value)
    
    dist_PID.update(distance)
    
    
    
#    print(1/(time.time() - timeCheck))
#    timeCheck = time.time()
    
# Cleanup before exit.
cv2.destroyAllWindows()
vs.stop()

