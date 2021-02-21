# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 17:39:34 2021

@author: KIMAIDE
"""
import numpy as np
from numpy import matmul as mul

class EKFSLAM:
    '''Class object for obtaining positional information using SLAM'''
    
    def __init__(self):
        
        self.N_landmarks = 15
        inf = 1E8
        
        self.u0 = np.zeros(3 + 2 * self.N_landmarks) #3 coordinate system + 2 * landmarks
        self.sigma0 = np.zeros([3 + 2 * self.N_landmarks, 3 + 2 * self.N_landmarks])
        
        self.Rt = np.zeros(3 + 2 * self.N_landmarks) #adjust added uncertainty as robot moves. Fix zero for now
        self.Gt = np.eye(3 + 2 * self.N_landmarks)
        
        for idx in range(3, 3 + self.N_landmarks):
            self.sigma0[idx, idx] = inf
            
            
    def Update (self, Vt, Wt, deltaT):
        '''update your robot's states and corresponding covariance as 
        the new position is updated'''
        
        self.u0[0] = self.u0[0] - Vt / Wt * np.sin(self.u0[2]) + \
                    Vt / Wt * np.sin(self.u0[2] + Wt * deltaT)
        self.u0[1] = self.u0[1] + Vt / Wt * np.cos(self.u0[2]) - \
                    Vt / Wt * np.cos(self.u0[2] + Wt * deltaT)
        self.u0[2] = self.u0[2] + Wt * deltaT
    
        #jacobian (affects only robot pos b/c that's the only updated)
        self.Gt[1, 3] = - Vt / Wt * np.cos(self.u0[2]) + \
                    Vt / Wt * np.cos(self.u0[2] + Wt * deltaT)
        self.Gt[2, 3] = - Vt / Wt * np.sin(self.u0[2]) + \
                    Vt / Wt * np.sin(self.u0[2] + Wt * deltaT)         
        
        self.sigma0 = mul(mul (self.Gt, self.sigma0), self.Gt.T) + self.Rt
    
    def correction (self, landmark):
        '''correct based on the measurement of new landmarks'''
        landmark = landmark
        
    def AssociateLandmark (self, Range, Bearing):
        z = np.array([Range, Bearing])
        
        #Q is uncertainty matrix of your lidar and bearing measurement
        Qt = np.array([[1, 0], [0, 1]])
        j = np.where(self.u0 == 0)[0][0]
        
        def mahalobis_distance(idx):
            Fxj = np.zeros([5, 3 + self.N_landmarks])
            Fxj[1,1] = 1;  Fxj[2,2] = 1; Fxj[0,0] = 1;
            Fxj[3, idx] = 1;  Fxj[4, idx+1] = 1
        
            '''Range from lidar measurement and Bearing from servo angle'''
            # ujx = self.u0[0] + Range * np.cos (Bearing + self.u0[2])
            # ujy = self.u0[0] + Range * np.sin (Bearing + self.u0[2])
            
            
            
            S = np.array([self.u0[0] - self.u0[idx],  self.u0[1] - self.u0[idx + 1]])
        
            q = mul(S.T, S)
            #expected observation
            z_expected = np.array([np.sqrt(q), np.arctan2(S[1], S[0]) - self.u0[2]])
            
            r = np.sqrt(q)
            H_low = np.array([[-S[0] / r, -S[1] / r, 0, S[0] / r, S[1] / r],
                             [S[1] / q, -S[1] / q, -q, -S[1] / q, S[0] / q]])
            
            H = mul(H_low, Fxj)
            #difference between predicted and actual observation
            e = (z - z_expected).T
            #calculate mahalobis distance of your new measurement
            cov_i =  np.linalg.inv(mul(mul(H, self.sigma0), H.T) + Qt)
            d = np.sqrt(mul(mul((z - z_expected).T, cov_i), (z - z_expected)))
            return d, cov_i, H, e
        
        alpha = 0.5
        mah_dist = [1E5]
        for i in range(3, j, 2):
            [d, cov_i, H] = mahalobis_distance(i)
            mah_dist.append(d)
            
        
        if mah_dist.min() > alpha:
            #new landmark
            self.u0[j] = self.u0[0] + Range * np.cos (Bearing + self.u0[2])
            self.u0[j+1] = self.u0[1] + Range * np.sin (Bearing + self.u0[2])
            [d, cov_i, H, e] = mahalobis_distance(j)
            
        elif mah_dist.min() < alpha:
            #existing landmark
            index_min = np.argmin(mah_dist)
            [d, cov_i, H, e] = mahalobis_distance(2 * (index_min - 1) + 3)

        
        Kt = mul(mul(self.sigma0, H.T), cov_i)
        
        self.u0 = self.u0 + mul(Kt, e)
        self.sigma0 = mul((np.identity(3 + 2 * self.N_landmarks) - mul(Kt, H)),  self.sigma0)


        
            
            
    




sigma = np.random.rand(15, 15)
np.random.seed(0)
sigma[11:, :] = 0 
sigma[:, 11:] = 0   
H_low = np.random.rand(2,5)  
np.random.seed(0)


Fxj = np.zeros([5,15])          
Fxj[1,1] = 1;  Fxj[2,2] = 1; Fxj[0,0] = 1;    
Fxj[3, 8] = 1;  Fxj[4, 9] = 1
H = mul(H_low, Fxj)
cov_i = mul(mul(H, sigma), H.T)

Kt = mul(mul(sigma, H.T), cov_i)