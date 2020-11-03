import RPi.GPIO as GPIO
import time
import RPi.GPIO as GPIO
import time

class clawControl:
    def __init__(self, pins):
        self.pins = pins #in1, in2, in3, in4
        self.open = False
        
        self.halfstep_seq = [
                              [1,0,1,0],
                              [1,0,0,0],
                              [1,0,0,1],
                              [0,0,0,1],
                              [0,1,0,1],
                              [0,1,0,0],
                              [0,1,1,0],
                              [0,0,1,0]
                            ]
        
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pins[0], GPIO.OUT); GPIO.setup(self.pins[1], GPIO.OUT);
        GPIO.setup(self.pins[2], GPIO.OUT); GPIO.setup(self.pins[3], GPIO.OUT); 
    
    #1.8 degrees / 2 for this half step control. Currently just set at 100
    def clawClose(self):
        for i in range(100):
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.halfstep_seq[halfstep][pin])
                time.sleep(0.001)
        self.open = False
        
        
    def clawOpen (self):
        for i in range(100):
            for halfstep in range(8, 0):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.halfstep_seq[halfstep][pin])
                time.sleep(0.001)
        self.open = True
        