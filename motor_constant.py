from motor_control import motorControl
import RPi.GPIO as GPIO 
import time
import Encoder
import pandas as pd

motor1 = motorControl([1, 7, 8])

enc = Encoder.Encoder(5,6)
time.sleep(2)
data = [[],[]]

i = 0
motor1.setPWM(0); motor1.backward()
startTime = time.time_ns()
while i < 50:
    motor1.setPWM(80); motor1.forward()
    data[0].append(time.time_ns() - startTime); data[1].append(enc.read())
    time.sleep(0.1)
    print(i, enc.read())
    i = i + 1


motor1.setPWM(0); motor1.stop()
#data = pd.DataFrame({'time':data[0], 'value':data[1]})
#data.to_csv('data.csv')