# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 08:09:52 2021

@author: KIMAIDE
"""

import numpy as np
from numpy import matmul as mul
from servo import ServoControl
import serial
from EKFSLAM import EKFSLAM
from sklearn.cluster import DBSCAN
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets
import time


class FindLandmark:
    def __init__(self):
        self.servo = ServoControl(26)
        self.slam = EKFSLAM()
        self.ser = serial.Serial("/dev/serial0", 115200)
        
    def getTFminiData(self):

        i = 0
        while i < 2500:
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


        for theta in range(-180, 180, 1):
            theta = theta / 4
            self.servo.TurnTo(theta)
            distance = 1E9
            while distance == 1E9:
                
                if self.ser.is_open == False:
                    self.ser.open()
                distance = self.getTFminiData()
            if distance > 2:
                continue 
            Ranges.append(distance)

            Bearings.append((-theta)%360)
            
        Ranges = Ranges[6:-6]
        Bearings = Bearings[6:-6]
        
        self.servo.TurnTo(0)

        

        xy = [self.slam.u0[0] + Ranges * np.cos ((Bearings + self.slam.u0[2]) * np.pi / 180), 
              self.slam.u0[1] + Ranges * np.sin ((Bearings + self.slam.u0[2]) * np.pi / 180)]
        xy = np.array(xy)

        min_point = 6 #15
        models = []
        plt.plot(xy[0], xy[1], 'o')
        r = np.sqrt(xy[0]**2 + xy[1]**2)
        theta = np.arctan2(xy[1], xy[0]) 
        xy[0] = r * np.cos(np.pi/4 + theta)
        xy[1] = r * np.sin(np.pi/4 + theta)

        
        division = 3
        for m in range(division):

            size = int(len(xy[0]) / division)
            x_sample = xy[0][m * size : (m + 1) * size]
            y_sample = xy[1][m * size : (m + 1) * size]
            ransac = linear_model.RANSACRegressor()
            ransac.fit(x_sample.reshape(-1, 1), y_sample)
            
            pred = ransac.predict(x_sample.reshape(-1, 1))
            
            error = abs(pred - y_sample)
            error = np.where(error < 0.075, True, error)
            error = np.where(error !=  True, False, error)

            if sum(error) > min_point:
                models.append(ransac)
        
        Landmarks = np.array([[0,0]])
        
        #determinig the balls (outliers) # may just take values as it is
        clustering = DBSCAN(eps=0.05, min_samples=4).fit(xy.T)
        
        self.xy = xy
        unique, counts = np.unique(clustering.labels_, return_counts = True)
        self.clustering = clustering
        for label in unique[counts < 20]:
            if label == -1:
                continue
            x_match = np.array(xy.T[clustering.labels_ == label]).T[0].mean()
            y_match = np.array(xy.T[clustering.labels_ == label]).T[1].mean()
            Landmarks = np.append(Landmarks, np.array([[x_match,y_match]]), axis = 0)
        
       
        
   


        if len(models) > 1:
            for i in range(len(models)-1):
                xrange = np.linspace(-0.5,2, 100000)
                df = pd.DataFrame({'x':xrange, 'y1':models[i].predict(xrange.reshape(-1,1)),
                                   'y2':models[i+1].predict(xrange.reshape(-1,1))})
                df['diff'] = abs(df['y1'] - df['y2'])
                
                if df['diff'].min() < 0.05:
                    df = df.loc[df['diff'] == df['diff'].min()]
                    if df['x'].values != None:
                        Landmarks = np.append(Landmarks, df[['x', 'y1']].values, axis=0)
                    
                    
        Landmarks = Landmarks[1:] 

        
        for i in range(len(Landmarks)):
            r = np.sqrt(Landmarks[i][0]**2 + Landmarks[i][1]**2)
            theta = np.arctan2(Landmarks[i][1], Landmarks[i][0]) 
            Landmarks[i][0] = r * np.cos(theta - np.pi/4)
            Landmarks[i][1] = r * np.sin(theta - np.pi/4)

        
        self.Landmarks = Landmarks
        
        print("landmarks from clustering", Landmarks)
        
        Ranges = np.array([]); Bearings = np.array([])
        for Landmark in Landmarks:
            R = np.sqrt(np.power(self.slam.u0[0] - Landmark[0], 2) + 
                          np.power(self.slam.u0[1] - Landmark[1], 2))
            Ranges = np.append(Ranges, R)
            
            B = np.arctan2(self.slam.u0[1] - Landmark[1],
                                       self.slam.u0[0] - Landmark[0])
            
        
            Bearings = np.append(Bearings, B)
        Bearings = Bearings * 180 / np.pi
        Bearings = Bearings % 360
        
        

        self.Ranges = Ranges; self.Bearings = Bearings
        for i, v in enumerate(zip(Ranges, Bearings)):
            print('measurements', i, v)
            if v[0] > 2:
                continue
            self.slam.AddLandmarks(v[0], v[1])
        


        


      