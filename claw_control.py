import RPi.GPIO as GPIO
import time
import RPi.GPIO as GPIO
import time

class clawControl:
    def __init__(self, pins):
        self.pins = pins #in1, in2, in3, in4
        self.open = False
        
        self.step_seq = [
                              [1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]
                            ]
        
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pins[0], GPIO.OUT); GPIO.setup(self.pins[1], GPIO.OUT);
        GPIO.setup(self.pins[2], GPIO.OUT); GPIO.setup(self.pins[3], GPIO.OUT); 
    
    #1.8 degrees / 2 for this half step control. Currently just set at 100
    def clawClose(self):
        for i in range(170):
            for step in range(4):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.step_seq[step][pin])
                time.sleep(0.01)
        self.open = False
        
        
    def clawOpen (self):
        for i in range(170):
            for step in range(4):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.step_seq[3-step][pin])
                time.sleep(0.01)
        self.open = True
if __name__ == "__main__":
    stepper = [27, 22, 23, 24] #in1, in2, in3, in4
    claw = clawControl(stepper)
    claw.clawClose()
    #time.sleep(2)
    #claw.clawOpen()
