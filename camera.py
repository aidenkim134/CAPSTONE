import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils

# Are we using the Pi Camera?
usingPiCamera = True
# Set initial frame size.
frameSize = (320, 240)
 
# Initialize mutithreading the video stream.
vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
        framerate=30).start()
# Allow the camera to warm up.
time.sleep(2.0)
 
#timeCheck = time.time()
began = False
while True:
    time.sleep(0.01)
    colorLimit = [([0,0,60], [60, 30, 225])]
    #colorLimit = [([50,50,0], [100, 100, 255])]
    colorLimit = [([0,0,80], [225, 70, 225])]
    colorLimit = [([30,0,60], [225, 60, 100])]
    colorLimit = [([0,0,80], [70, 20, 255])]
    # Get the next frame.
    vs.camera.zoom = (0.45, 0.45, 0.45, 0.45)
    frame = vs.read()

    cv2.imwrite('original.png', frame)
    #only take red or blue color
    for (lower, upper) in colorLimit:
        lower = np.array(lower, dtype='uint8')
        upper = np.array(upper, dtype='uint8')
        
        mask = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        
    #find circular image using edge finding and hough transform (circle)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 4.5, 40)
    if began == False:
        preCircle = np.array([[0,0,0]])
        began = True
    
    try:
        circles = np.round(circles[0, :]).astype(int)
        preCircle = circles.copy()
        color = 225
    except:
        circles = preCircle
        color = 100
    print(circles)  
    for (x, y, r) in circles:
        cv2.circle(frame, (x,y), r, (0, color, 0), 4)
        cv2.rectangle(frame, (x-2, y-2), (x+2, y+2), (0, 128, 225), -1)
        break
    
    # Show video stream
    cv2.imshow('orig', frame)
    key = cv2.waitKey(1) & 0XFF
    
    


    
#    print(1/(time.time() - timeCheck))
#    timeCheck = time.time()
    
# Cleanup before exit.
cv2.destroyAllWindows()
vs.stop()

