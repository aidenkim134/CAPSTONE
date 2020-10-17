# -*- coding: utf-8 -*
import serial
import time

ser = serial.Serial("/dev/serial0", 115200)

def getTFminiData():
    while True:
        time.sleep(0.1)
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)   
            ser.reset_input_buffer() 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                distance = distance / 100
                print("current distance is {}m".format(distance))
                ser.reset_input_buffer()
                
if __name__ == '__main__':
    try:
        if ser.is_open == False:
            ser.open()
        getTFminiData()
    except KeyboardInterrupt:   
        if ser != None:
            ser.close()