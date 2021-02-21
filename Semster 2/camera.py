from picamera import PiCamera
import time
from picamera.array import PiRGBArray
import cv2

camera = PiCamera()
camera.rotation = 180

rawCapture = PiRGBArray(camera)
time.sleep(0.1)

camera.capture(rawCapture, format='bgr')
image = rawCapture.array
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#imwrite

cv2.imshow("Image", image)
cv2.waitKey(0)