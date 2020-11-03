import RPi.GPIO as GPIO
import time

class motorControl:
    def __init__(self, motor):
        self.motor = motor #en, out1, out2
        self.frequency = 1000
        self.rotation = 0
        
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.motor[0], GPIO.OUT); GPIO.setup(self.motor[1], GPIO.OUT);
        GPIO.setup(self.motor[2], GPIO.OUT); 
                
        self.speed = GPIO.PWM(self.motor[0], self.frequency) 

        self.speed.start(0);
        
    def setPWM(self, pwm):
        self.pwm = pwm

    def stop(self):
        GPIO.output(self.motor[1], GPIO.LOW); GPIO.output(self.motor[2], GPIO.LOW)
        self.speed.ChangeDutyCycle(0);
    
    def forward(self):
        GPIO.output(self.motor[1], GPIO.HIGH); GPIO.output(self.motor[2], GPIO.LOW)
        self.speed.ChangeDutyCycle(self.pwm);
    
    def backward(self):
        GPIO.output(self.motor[1], GPIO.LOW); GPIO.output(self.motor[2], GPIO.HIGH)
        self.speed.ChangeDutyCycle(self.pwm);
    

            