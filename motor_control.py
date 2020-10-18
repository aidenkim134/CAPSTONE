import RPi.GPIO as GPIO
import time

class motorControl:
    def __init__(self, motor)
        self.motor = motor #out1, out2, in1, in2,
        self.frequency = 10000
        
    def setPWM(self, pwm):
        self.pwm = pwm
        
    def setPins(self):
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.motor[0] ,GPIO.IN); GPIO.setup(self.motor[1] ,GPIO.IN);
        
        GPIO.setup(self.motor[2], GPIO.OUT); GPIO.setup(self.motor[3], GPIO.OUT)

    
        self.OUT1 = GPIO.PWM(self.motor[0], frequency) #left
        self.OUT2 = GPIO.PWM(self.motor[1], frequency) #left
        self.OUT1.start(0); self.OUT2.start(0);

    def stop(self):
        self.OUT1.ChangeDutyCycle(0); self.OUT2.ChangeDutyCycle(0);
    
    def forward(self):
        self.OUT1.ChangeDutyCycle(self.pwm); self.OUT2.ChangeDutyCycle(0)
    
    def backward(self):
        self.OUT1.ChangeDutyCycle(0); self.OUT2.ChangeDutyCycle(self.pwm)
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
            