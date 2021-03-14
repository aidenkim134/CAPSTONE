# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 05:32:06 2021

@author: KIMAIDE
"""
import numpy as np
from scipy.spatial import distance_matrix
import pandas as pd
from math import sqrt, acos, atan2, sin, cos
import matplotlib.pyplot as plt
class PathPlanning:
    def __init__(self, u0, destination):
        fig = plt.figure(num=None, figsize=(8,8), dpi=80, facecolor='w', edgecolor='k')
        self.ax = fig.add_subplot(1, 1, 1)
        
        self.pos = u0[0:2]
        x = []; y = []
        for idx in range(3, len(u0), 2):
            x.append(u0[idx])
            y.append(u0[idx + 1])
        
        self.x = np.array(x); self.y = np.array(y)
        r = np.sqrt(self.x**2 + self.y**2)
        
        idxmax = r.argmax(); idxmin = r.argmin()
        
        if destination == 'max':
            self.xloc = x[idxmax]; self.yloc = y[idxmax]
            
        elif destination == 'min':
            self.xloc = x[idxmin]; self.yloc = y[idxmin]
 
        
        size = len(self.x)
        self.x = np.append(self.x, [np.repeat(self.x[idxmax], size), 
                                            np.repeat(self.x[idxmin], size)])
        self.y = np.append(self.y, [self.y, self.y])
        
        self.y = np.append(self.y, [np.repeat(self.y[idxmax], size), 
                                        np.repeat(self.y[idxmin], size)])        
        self.x = np.append(self.x, [self.x[:size], self.x[:size]])
        
        self.ax.plot(self.x[size:], self.y[size:], 'o')
        
        
        self.ax.plot(self.x[:size], self.y[:size], 'o');

        self.ax.plot(self.pos[0], self.pos[1], 'd')

        df = pd.DataFrame({'x':self.x, 'y':self.y})
        distmat = pd.DataFrame(distance_matrix(df.values, df.values))
        
        self.radius = 0.20
        self.Dcircle = 0.1
        self.distmat = distmat.apply(lambda x: x > 0.20 + 0.1 * 2) 
        
        if destination == 'max':
            self.xloc = self.xloc - 0.25; self.yloc = self.yloc - 0.25
            
        elif destination == 'min':
            self.xloc = self.xloc + 0.25; self.yloc = self.yloc + 0.25
        
        self.ax.plot(self.xloc, self.yloc, 'x')
        
        
    def getPoints (self):
        def validate (xv, yv):
            FoundPath = False
            df = pd.DataFrame({'xv':xv, 'yv':yv})
            validate = pd.DataFrame(distance_matrix(df_crit.values, df.values))
            validate = validate.apply(lambda x: x < 0.01)
            if validate.sum().sum() < 1:
                self.p = np.array([self.xloc, self.yloc])
                FoundPath = True
            return FoundPath
            

        x_crit = np.array([]); y_crit = np.array([])
        for idx, row in self.distmat.iterrows():
            omit = self.distmat.index[self.distmat[idx] == False].tolist()
            omit.remove(idx)
            
            for loc in omit:
                x_crit = np.append(x_crit, np.linspace(self.x[idx], self.x[loc]))
                y_crit = np.append(y_crit, np.linspace(self.y[idx], self.y[loc]))
                
                
        self.dangerCircle()
        x_crit = np.append(x_crit, self.danger[0])  
        y_crit = np.append(y_crit, self.danger[1])  
        df_crit = pd.DataFrame({'x_crit':x_crit, 'y_crit':y_crit})

        # 1 point
        xv = np.linspace(self.pos[0], self.xloc, 1000)
        yv = np.linspace(self.pos[1], self.yloc, 1000)
        
        FoundPath = validate(xv, yv)

        if FoundPath == True:
            self.ax.plot(self.xloc, self.yloc, 'd')
            return [[self.xloc, self.yloc]]
            
        # n point
        
        min_idx = 0; min_dist = 1E6
        for idx in range(len(self.x)):
            dxs = self.x[idx] - xv
            dys = self.y[idx] - yv
            
            d = np.sqrt(dxs**2 + dys**2)

            if d.min() < min_dist:
                min_dist = d.min()
                self.min_dist = min_dist
                min_idx = idx
        
        To = self.tangent(self.pos[0], self.pos[1], self.x[min_idx], self.y[min_idx])
        ddo = pd.DataFrame({'x':To[0], 'y':To[1]})

        Tloc = self.tangent(self.xloc, self.yloc, self.x[min_idx], self.y[min_idx])
        ddl = pd.DataFrame({'x':Tloc[0], 'y':Tloc[1]})

        dd = pd.DataFrame(distance_matrix(ddo.values, ddl.values))
        dd = dd.idxmin() == dd.index
        
        if dd[0] == False:
            To[0] = np.flipud(To[0])
            To[1] = np.flipud(To[1])

        self.ax.plot(To[0][0], To[1][0], 'd', markersize = 2) 
        self.ax.plot(To[0][1], To[1][1], 'd', markersize = 2)
        self.ax.plot(Tloc[0][0], Tloc[1][0], 'd', markersize = 2)
        self.ax.plot(Tloc[0][1], Tloc[1][1], 'd', markersize = 2)

        lino1 = self.line([To[0][0], self.pos[0]], [To[1][0], self.pos[1]])
        
        
        lino2 = self.line([To[0][1], self.pos[0]], [To[1][1], self.pos[1]])


        linp1 = self.line([Tloc[0][0], self.xloc], [Tloc[1][0], self.yloc])

        linp2 = self.line([Tloc[0][1], self.xloc], [Tloc[1][1], self.yloc])

        x1  = (lino1[0] - linp1[0]) / (linp1[1] - lino1[1])
        y1 = lino1[0] + lino1[1] * x1

        x2  = (lino2[0] - linp2[0]) / (linp2[1] - lino2[1])
        y2 = lino2[0] + lino2[1] * x2
        
        x = [x1, x2]; y = [y1, y2]
        self.ax.plot(x, y, 'p', markersize = 2)

        xv = [np.array([]), np.array([])]; yv = [np.array([]), np.array([])]     

        for i in range(2):
            xv[i] = np.linspace(self.pos[0], x[i])
            xv[i] = np.append(xv[i], np.linspace(x[i], self.xloc))
                                 
            yv[i] = np.linspace(self.pos[1], y[i])
            yv[i] = np.append(yv[i], np.linspace(y[i], self.yloc))                 
            
        for i in range(2):
            FoundPath = validate(xv[i], yv[i])
            if FoundPath == True:
                self.ax.plot(x[i], y[i], 'd', self.xloc, self.yloc, 'd')
                return [[x[i], y[i]], [self.xloc, self.yloc]]
            
        
    def dangerCircle(self):
        
        def PointsInCircum(r, n=100):
            return np.array([(np.cos(2 * np.pi / n * x) * r, 
                              np.sin(2 * np.pi / n * x) * r) for x in range(0, n+1)]) 
        
        danger = np.array([[],[]]).T
        R = np.linspace(1E-6, self.Dcircle-0.01, num = 10)
        for i in range(len(self.x)):
            p = np.array([self.x[i], self.y[i]])
            for r in R:
                danger = np.append(danger, PointsInCircum(r) + p, axis=0)
                
        self.danger = danger.T
        self.ax.plot(self.danger[0], self.danger[1], 'o', markersize=0.1)
        
            
    def tangent(self, Px, Py, Cx, Cy):
        a = self.Dcircle
        b = np.sqrt((Px - Cx)**2 + (Py - Cy)**2)  
        th = np.arccos(a / b)  # angle theta
        d = np.arctan2(Py - Cy, Px - Cx)  # direction angle of point P from C
        d1 = d + th  # direction angle of point T1 from C
        d2 = d - th  # direction angle of point T2 from C
        
        x = np.array([Cx + a * np.cos(d1), Cx + a * np.cos(d2)])
        y = np.array([Cy + a * np.sin(d1), Cy + a * np.sin(d2)])
        
        return [x, y]
    
    def line(self, x, y):
        m = (y[1] - y[0]) / (x[1] - x[0])
        c = -m * x[0] + y[0]
        
        return [c, m]

        

if __name__ == '__main__':
    p = np.array([0.5,0.75,3,0,0,0, 1.8, 1.8, 1.8, 1.8, 0])  
    p = np.append(p, np.array([0.4, 0.6, 0.7, 0.9, 1.6, 1.3, 0.5, 1.3]))
    
    obj = PathPlanning(p, 'max')   
    point = obj.getPoints()  
        
        
        
        
        

            
        