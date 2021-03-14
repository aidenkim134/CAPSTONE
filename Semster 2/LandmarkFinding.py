# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 08:09:52 2021

@author: KIMAIDE
"""

import numpy as np
from numpy import matmul as mul
from servo import ServoControl
import serial
import EKFSLAM
from sklearn.cluster import DBSCAN
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets

class FindLandmark:
    def __init__(self):
        self.servo = ServoControl(26)
        self.slam = EKFSLAM()

        self.ser = serial.Serial("/dev/serial0", 115200)
    def getTFminiData(self):
        i = 1
        while i < 500:
            count = self.ser.in_waiting
            if count > 8:
                recv = self.ser.read(9)   
                self.ser.reset_input_buffer() 
                
                if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                    distance = recv[2] + recv[3] * 256
                    distance = distance / 100
                    #print("current distance is {}m".format(distance))
                    self.ser.reset_input_buffer()
                    return distance  
            i = i + 1
        return 1E9
    
    def Store(self):
        Ranges = []; Bearings = []
        for theta in range(-45, 45, 1):
            self.servo.TurnTo(theta)
            
            distance = self.getTFminiData()
            Ranges.append(distance)
            
            Bearings.append((self.slam.u0[2] + theta)%360)
        
        self.servo.TurnTo(0)
        
        xy = [self.slam.u0[0] + Ranges * np.cos ((Bearings + self.u0[2]) * np.pi / 180), 
              self.slam.u0[0] + Ranges * np.cos ((Bearings + self.u0[2]) * np.pi / 180)]
        
        #finding corners
        
        n_points = 10

        min_point = 15
        models = []
        for corner in range(5):

            idx = np.random.randint(0, len(xy) - n_points)
            ransac = linear_model.RANSACRegressor()
            ransac.fit(xy[0, idx:idx+n_points].reshape(-1, 1), xy[1, idx:idx+n_points])
            pred = ransac.predict(xy[0].reshape(-1, 1))
            error = abs(pred - xy[1])
            error = np.where(error <= 5, error, True)
            error = np.where(error > 5, error, False)
            
            if sum(error) > min_point:
                xy = xy[:, error == 0]
                models.append(ransac)
                
            if len(models) > 1:
                #intecepting points as landmarks
                xrange = np.linspace(-1, 10)
                df = pd.DataFrame({'x':xrange, 'y1':models[0].predict(xrange),
                                   'y2':models[1].predict(xrange)})
                df['diff'] = abs(df['y1'] - df['y2'])
                df = df.loc[df['diff'] == df['diff'].min()]
                
                break

        #determinig the balls (outliers) # may just take values as it is
        clustering = DBSCAN(eps=3, min_samples=2).fit(xy.T)
        landmark = xy.T[clustering.labels_ == -1].T
        
        landmark = np.append(landmark, df[['x', 'y1']].values, axis = 1)
        
        Ranges = np.sqrt(np.power(self.u0[0]^2 - landmark[0], 2) + 
                          np.power(self.u0[1]^2 - landmark[1], 2))
        
        Bearings = np.arctan2(self.u0[1]^2 - landmark[1],
                                       self.u0[0]^2 - landmark[0])
            
        
        for Range, Bearing in enumerate(zip(Ranges, Bearings)):
            self.slam.AssociateLandmark(Range, Bearing)

        



      