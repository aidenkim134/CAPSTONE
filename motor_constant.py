from motor_control import motorControl
import RPi.GPIO as GPIO 
import time
import Encoder
import pandas as pd

motor1 = motorControl([21, 20, 16])

enc = Encoder.Encoder(13,19)
time.sleep(2)
data = [[],[]]

i = 0
motor1.setPWM(100); motor1.forward()

while i < 1000:
    data[0].append(i * 0.001); data[1].append(enc.read())
    time.sleep(0.001)
    print(i)
    i = i + 1

print(len(data[0]), len(data[1]))
motor1.setPWM(0); motor1.stop()
data = pd.DataFrame({'time':data[0], 'value':data[1]})
data.to_csv('data.csv')