# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:10:07 2020

@author: KIMAIDE
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('data.csv')
data['dt'] = np.append(0,np.diff(data['time']))
data['ds'] = np.append(0,np.diff(data['value']))
plt.figure()
plt.plot(data['time'] / 1E9,data['ds'] / (data['dt']/1E9)/3531 * 2* np.pi)
plt.xlabel('time(s)'); plt.ylabel('rad/s')
plt.title('Motor speed response to input of [20]')
plt.show()
plt.savefig('motorConstant.png')