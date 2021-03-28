from picamera import PiCamera
import time
from picamera.array import PiRGBArray
import cv2
camera = PiCamera()
camera.rotation = 180
camera.resolution = (224, 224)
rawCapture = PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawCapture, format='bgr')
image = rawCapture.array

#imwrite
cv2.imshow("Image", image)
cv2.waitKey(0)