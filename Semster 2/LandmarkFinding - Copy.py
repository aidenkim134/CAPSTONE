# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 08:09:52 2021

@author: KIMAIDE
"""

import numpy as np
from numpy import matmul as mul
#from servo import ServoControl
#import serial
from EKFSLAM import EKFSLAM
from sklearn.cluster import DBSCAN
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import linear_model, datasets
import time

class FindLandmark:
    def __init__(self):
        # self.servo = ServoControl(26)
        self.slam = EKFSLAM()
        # self.ser = serial.Serial("/dev/serial0", 115200)
        
    def getTFminiData(self):
        i = 0
        while i < 2E3:
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
        
    def Store(self, xy):
        Ranges = []; Bearings = []

        '''
        for theta in range(-45, 45, 10):
            self.servo.TurnTo(theta)
            distance = 1E9
            while distance == 1E9:
                if self.ser.is_open == False:
                    self.ser.open()
                distance = self.getTFminiData()
            Ranges.append(distance)
            print(distance)

            Bearings.append((self.slam.u0[2] + theta)%360)
            

        
        self.servo.TurnTo(0)
        self.servo.servo.stop()
        

        xy = [self.slam.u0[0] + Ranges * np.cos ((Bearings + self.slam.u0[2]) * np.pi / 180), 
              self.slam.u0[0] + Ranges * np.cos ((Bearings + self.slam.u0[2]) * np.pi / 180)]
        xy = np.array(xy)
        '''
        
        #finding corners
        

        min_point = 4 #15
        models = []
        
        r = np.sqrt(xy[0]**2 + xy[1]**2)
        theta = np.arctan2(xy[1], xy[0]) 
        xy[0] = r * np.cos(np.pi/4 + theta)
        xy[1] = r * np.sin(np.pi/4 + theta)
        plt.plot(xy[0], xy[1], 'o')
        
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
        clustering = DBSCAN(eps=0.2, min_samples=2).fit(xy.T)
        Landmarks = np.append(Landmarks, np.array(xy.T[clustering.labels_ == -1]), axis = 0)

        if len(models) > 1:
            for i in range(len(models)-1):
                xrange = np.linspace(-2.5,2.5, 100000)
                df = pd.DataFrame({'x':xrange, 'y1':models[i].predict(xrange.reshape(-1,1)),
                                   'y2':models[i+1].predict(xrange.reshape(-1,1))})
                df['diff'] = abs(df['y1'] - df['y2'])
                
                if df['diff'].min() < 0.05:
                    df = df.loc[df['diff'] == df['diff'].min()]
                    if df['x'].values != None:
                        Landmarks = np.append(Landmarks, df[['x', 'y1']].values, axis=0)
                    
                    
        Landmarks = Landmarks[1:] 
        
        self.Landmarks = Landmarks
        
        for i in range(len(Landmarks)):
            r = np.sqrt(Landmarks[i][0]**2 + Landmarks[i][1]**2)
            theta = np.arctan2(Landmarks[i][1], Landmarks[i][0]) 
            Landmarks[i][0] = r * np.cos(theta - np.pi/4)
            Landmarks[i][1] = r * np.sin(theta - np.pi/4)
        
        Landmarks = np.append(Landmarks, Landmarks, axis=0)
        
        Ranges = np.array([])
        for Landmark in Landmarks:
            R = np.sqrt(np.power(self.slam.u0[0]**2 - Landmark[0], 2) + 
                          np.power(self.slam.u0[1]**2 - Landmark[1], 2))
            Ranges = np.append(Ranges, R)
            
            B = np.arctan2(-self.slam.u0[1]**2 + Landmark[1],
                                       -self.slam.u0[0]**2 + Landmark[0])
            
        
            Bearings = np.append(Bearings, B)
        Bearings = Bearings * 180 / np.pi     
        self.Ranges = Ranges; self.Bearings = Bearings
        for i, v in enumerate(zip(Ranges, Bearings)):
            self.slam.AssociateLandmark(v[0], v[1])
        
        # for corner in range(5):
            
        #     idx = np.random.randint(0, len(xy[0]) - n_points)

        #     ransac = linear_model.RANSACRegressor()
        #     ransac.fit(xy[0][idx:idx+n_points].reshape(-1, 1), xy[1][idx:idx+n_points])
        #     pred = ransac.predict(xy[0].reshape(-1, 1))
        #     error = abs(pred - xy[1])
        #     error = np.where(error <= 5, error, True)
        #     error = np.where(error > 5, error, False)
            
        #     if sum(error) > min_point:
        #         xy = xy[:, error == 0]
        #         models.append(ransac)
                
        #     if len(models) > 1:
        #         #intecepting points as landmarks
        #         xrange = np.linspace(-1, 10)
        #         df = pd.DataFrame({'x':xrange, 'y1':models[0].predict(xrange),
        #                            'y2':models[1].predict(xrange)})
        #         df['diff'] = abs(df['y1'] - df['y2'])
        #         df = df.loc[df['diff'] == df['diff'].min()]
                
        #         break
        
        # #determinig the balls (outliers) # may just take values as it is
        # clustering = DBSCAN(eps=3, min_samples=2).fit(xy.T)
        # landmark = xy.T[clustering.labels_ == -1].T
        
        
        
        # #landmark = np.append(landmark, df[['x', 'y1']].values, axis = 1)
        
        # Ranges = np.sqrt(np.power(self.slam.u0[0]**2 - landmark[0], 2) + 
        #                   np.power(self.slam.u0[1]**2 - landmark[1], 2))
        
        # Bearings = np.arctan2(self.slam.u0[1]**2 - landmark[1],
        #                                self.slam.u0[0]**2 - landmark[0])
            
        
        # for Range, Bearing in enumerate(zip(Ranges, Bearings)):
        #     self.slam.AssociateLandmark(Range, Bearing)

        
if __name__ == '__main__':
    u0 = np.array([0.5,0.75,3,0,0,0, 1.8, 1.8, 1.8, 1.8, 0])  
    u0 = np.append(u0, np.array([0.4, 0.6, 0.7, 0.9, 1.6, 1.3, 0.5, 1.3]))

    x = []; y = []
    for idx in range(3, len(u0), 2):
        x.append(u0[idx])
        y.append(u0[idx + 1])
    x = np.array(x); y = np.array(y)
    x = np.linspace(0, 1,10)   
    y = np.zeros(10)
    size = len(x)
    r = np.sqrt(x**2 + y**2)
    idxmax = r.argmax(); idxmin = r.argmin()    
    x = np.array(x); y = np.array(y)
    x = np.append(x, np.repeat(1, size))
    y = np.append(y, x[:size])
    
    y = np.append(y, np.repeat(1, size))    
    x = np.append(x, x[:size]) 
    for i in range(len(x)):
        x[i] = x[i] + np.random.randint(0,9) / 100   
        y[i] = y[i] + np.random.randint(0,9) / 100   
    
    x[-1] = 0.5; y[-1] = 0.5
    
    obj = FindLandmark()
    xy = np.array([x, y])
    obj.Store(xy)
      