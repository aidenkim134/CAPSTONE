# -*- coding: utf-8 -*
import serial
import time

ser = serial.Serial("/dev/serial0", 115200)

def getTFminiData():
    i = 0
    while i < 2000:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)   
            ser.reset_input_buffer() 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                distance = distance / 100
                #print("current distance is {}m".format(distance))
                ser.reset_input_buffer()
                return distance  
        i = i + 1
    return 1E9
            
if __name__ == '__main__':
    try:
        while True:
            time.sleep(0.01)
            if ser.is_open == False:
                ser.open()
            print(getTFminiData())
    
    except KeyboardInterrupt:   
        if ser != None:
            ser.close()