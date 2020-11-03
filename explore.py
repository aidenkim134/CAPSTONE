# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:10:07 2020

@author: KIMAIDE
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('data.csv')

data['ds'] = np.append(0,np.diff(data['value']))
plt.figure()
plt.plot(data['time'],data['ds'] / 0.001, 'd')
plt.ylim([0,10000]);