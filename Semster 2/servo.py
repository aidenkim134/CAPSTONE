# Import libraries
import RPi.GPIO as GPIO
import time

class ServoControl:
    def __init__(self, pin):
        frequency = 50
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(pin, GPIO.OUT)
        self.servo = GPIO.PWM(pin, frequency) # Note 11 is pin, 50 = 50Hz pulse
    
        #start PWM running, but with value of 0 (pulse off)
        self.servo.start(0)
        time.sleep(1)
     
    def TurnTo (self, angle):

        #angle given in theta
        angle = angle + 90
        #2 to 12 is 0 to 180 degrees: f = 2 + 10 * theta / 180 
        f = lambda x: 2 + 10 * x / 180
        self.servo.ChangeDutyCycle(f(angle))
        
        time.sleep(0.1)
    
        self.servo.ChangeDutyCycle(0)

  
        
        
if __name__ == '__main__':
    servo = ServoControl(26)
    
    for i in range(-45, 45, 5):
        time.sleep(0.1)
        servo.TurnTo(i)

    servo.TurnTo(0)
    


