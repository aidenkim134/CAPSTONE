# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:10:07 2020

@author: KIMAIDE
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat
from scipy.special import erfc
from scipy.optimize import least_squares
from control import matlab
data = pd.read_csv('data.csv')
data['time'] = data['time'] / 1E9
data['dt'] = np.append(0,np.diff(data['time'])) 
data['speed'] = np.append(0,np.diff(data['value'])) / data['dt'] / 3531 * 2 * np.pi
data = data.fillna(0)

plt.figure()
plt.plot(data['time'] , data['speed'])
plt.xlabel('time(s)'); plt.ylabel('rad/s')
plt.title('Motor speed response to input of [20]')
plt.show()
plt.savefig('motorConstant.PNG')

class curveFit2:
    def __init__ (self, data):
        self.data = data
        self.time = np.linspace(data['time'].min(), data['time'].max(), 51)
    def function (self, params):
        a, b, c, d = params
        g = matlab.tf(a, [b, c, d])
        [y, t] = matlab.step(g, self.time)
        return y - self.data['speed']
                
    def solve(self):
        self.sol = least_squares(self.function, x0 = [0.1]* 4)
        self.params = self.sol.x
        
        self.g = matlab.tf(self.params[0], [self.params[1],self.params[2], self.params[3]])
        [y, t] = matlab.step(self.g, self.time)
        plt.figure(num=None, figsize=(8, 6), dpi=80, facecolor='w', edgecolor='k')
        plt.plot(self.data['time'], self.data['speed'])
        plt.plot(self.time, y)
        plt.xlim([0,5])

obj = curveFit2(data)
obj.solve()



    

