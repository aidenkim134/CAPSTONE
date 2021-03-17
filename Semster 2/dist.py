import RPi.GPIO as GPIO
import time

class DistanceM:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.pinTrigger = 14
        self.pinEcho = 15
        GPIO.setup(self.pinTrigger, GPIO.OUT)
        GPIO.setup(self.pinEcho, GPIO.IN)        

    def getDist(self):

        
        GPIO.output(self.pinTrigger, GPIO.LOW)
        GPIO.output(self.pinTrigger, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.pinTrigger, GPIO.LOW)
     
        while GPIO.input(self.pinEcho)==0:
            pulseStartTime = time.time()
        while GPIO.input(self.pinEcho)==1:
            pulseEndTime = time.time()
     
        pulseDuration = pulseEndTime - pulseStartTime
        distance = round(pulseDuration * 17150, 2) / 100
        return distance

