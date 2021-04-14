'''This script is used to take care of SLAM using Extended Kalman Filter'''
'''Refer to SLAM for dummies for calculation procedures'''
import numpy as np
from numpy import matmul as mul

class EKFSLAM:
    '''Class object for obtaining positional information using SLAM'''
    def __init__(self):
        
        self.N_landmarks = 40
        inf = 1E8
        
        self.u0 = np.zeros(3 + 2 * self.N_landmarks) #3 coordinate system + 2 * landmarks
        self.sigma0 = np.zeros([3 + 2 * self.N_landmarks, 3 + 2 * self.N_landmarks])
        self.u0[1] = 0.65; 
        self.Rt = np.zeros([3 + 2 * self.N_landmarks, 3 + 2 * self.N_landmarks]) #adjust added uncertainty as robot moves. Fix zero for now
        self.Rt[0,0] = 0.0; self.Rt[1,1] = 0.0; self.Rt[2,2] = 0.0; 
        self.Gt = np.eye(3 + 2 * self.N_landmarks)
        
        for idx in range(3, 3 + self.N_landmarks):
            self.sigma0[idx, idx] = inf
            
            
    def Update (self, Vt, Wt, deltaT):
        '''update your robot's states and corresponding covariance as 
        the new position is updated'''

        if Wt != 0:
            self.u0[0] = self.u0[0] - Vt / Wt * np.sin(self.u0[2] * np.pi / 180) + \
                        Vt / Wt * np.sin((self.u0[2] * np.pi / 180 + Wt * deltaT))
            self.u0[1] = self.u0[1] + Vt / Wt * np.cos(self.u0[2] * np.pi / 180) - \
                        Vt / Wt * np.cos((self.u0[2] * np.pi / 180 + Wt * deltaT))
            self.u0[2] = self.u0[2] + Wt * deltaT * 180 / np.pi
            
            self.u0[2] = self.u0[2] % 360
        
            #jacobian (affects only robot pos b/c that's the only updated)
            self.Gt[1, 3] = - Vt / Wt * np.cos(self.u0[2] * np.pi / 180) + \
                        Vt / Wt * np.cos((self.u0[2] * np.pi / 180 + Wt * deltaT))
            self.Gt[2, 3] = - Vt / Wt * np.sin(self.u0[2] * np.pi / 180) + \
                        Vt / Wt * np.sin((self.u0[2] * np.pi / 180 + Wt * deltaT))
        elif Wt == 0:
            '''for certain situation, right and wheel wheel distance would be 
            same for the given time interval'''
            self.u0[0] = self.u0[0] + Vt * np.cos(self.u0[2] * np.pi / 180) * deltaT
            self.u0[1] = self.u0[1] + Vt * np.sin(self.u0[2] * np.pi / 180) * deltaT    
            self.u0[2] = self.u0[2] % 360

            self.Gt[1, 3] = Vt * np.sin(self.u0[2] * np.pi / 180) * deltaT
                        
            self.Gt[2, 3] = Vt * np.cos(self.u0[2] * np.pi / 180) * deltaT
                        
        self.sigma0 = mul(mul (self.Gt, self.sigma0), self.Gt.T) + self.Rt
    
    def AddLandmarks(self, Range, Bearing):
        
        '''add landmarks based on euclidean association'''
        x = self.u0[0] + Range * np.cos ((Bearing + self.u0[2]) * np.pi / 180)
        y = self.u0[1] + Range * np.sin ((Bearing + self.u0[2]) * np.pi / 180)
        j = np.where(self.u0[3:] == 0)[0][0] + 3
        new = True
        for idx in range(3, j, 2):
            if np.sqrt((self.u0[idx]-x)**2 + (self.u0[idx + 1] - y)**2)  < 0.2:
                self.u0[idx] = (self.u0[idx] + x) / 2
                self.u0[idx + 1] = (self.u0[idx + 1] + y) / 2
                new = False
                break
        if new:
            self.u0[j] = x
            self.u0[j+1] = y
            
        self.closest()
        
            
        
    def AssociateLandmark (self, Range, Bearing):
        
        '''Add landmark based on extended kalman filter association'''
        z = np.array([Range, Bearing])
        
        #Q is uncertainty matrix of your lidar and bearing measurement
        Qt = np.array([[1, 0], [1,0]])
        j = np.where(self.u0[3:] == 0)[0][0] + 3
        
        def mahalobis_distance(idx):
            Fxj = np.zeros([5, 3 + 2 * self.N_landmarks])
            Fxj[0,0] = 1; Fxj[1,1] = 1;  Fxj[2,2] = 1;
            Fxj[3, idx] = 1;  Fxj[4, idx+1] = 1
        
            '''Range from lidar measurement and Bearing from servo angle'''
            # ujx = self.u0[0] + Range * np.cos (Bearing + self.u0[2])
            # ujy = self.u0[0] + Range * np.sin (Bearing + self.u0[2])
            
            S = np.array([self.u0[0] - self.u0[idx],  self.u0[1] - self.u0[idx + 1]]) * -1
        
            q = mul(S.T, S)

            #expected observation
            z_expected = np.array([np.sqrt(q), np.arctan2(S[1], 
                                    S[0]) * 180 / np.pi - self.u0[2]])
            z_expected[1] = z_expected[1] % 360

            r = np.sqrt(q)
            H_low = np.array([[-S[0] / r, -S[1] / r, 0, S[0] / r, S[1] / r],
                             [S[1] / q, -S[1] / q, -q, -S[1] / q, S[0] / q]])
            # define jacobian
            H = mul(H_low, Fxj)
            
            #difference between predicted and actual observation
            e = (z - z_expected).T
            e = abs(e)
            
            
            #calculate mahalobis distance of your new measurement
            cov_i =  np.linalg.inv(mul(mul(H, self.sigma0), H.T) + Qt)
            d = np.sqrt(mul(mul(e, cov_i), e.T))
            return d, cov_i, H, e
        
        ''''add new landmark or associate existing landmark based on 
        distance'''
        alpha = 3
        mah_dist = [1E5]
        for i in range(3, j, 2):
            [d, cov_i, H, e] = mahalobis_distance(i)
            mah_dist.append(d)
        mah_dist = np.array(mah_dist)   
        
        if mah_dist.min() > alpha:
            #new landmark
            self.u0[j] = self.u0[0] + Range * np.cos ((Bearing + self.u0[2]) * np.pi / 180)
            self.u0[j+1] = self.u0[1] + Range * np.sin ((Bearing + self.u0[2]) * np.pi / 180)
            [d, cov_i, H, e] = mahalobis_distance(j)

            
        elif mah_dist.min() < alpha:
            #existing landmark
            index_min = np.argmin(mah_dist)
            [d, cov_i, H, e] = mahalobis_distance(2 * (index_min - 1) + 3)

 
        
        Kt = mul(mul(self.sigma0, H.T), cov_i)

        check = np.where(self.u0, abs(self.u0) > 2, 0)
        if sum(check) == 0:
            self.u0 = self.u0 + mul(Kt, e)
            self.sigma0 = mul((np.identity(3 + 2 * self.N_landmarks) - mul(Kt, H)),  self.sigma0)

    def closest (self):
        '''determine the closest distance ball'''
        self.ClosestIdx = 3
        maxDist = 10000
        offset = 0.3; maxp = 1.6
        for idx in range(3, len(self.u0), 2):
            if (self.u0[idx] != 0 and self.u0[idx] > offset
                    and self.u0[idx +1] > offset and self.u0[idx] < maxp-offset and  self.u0[idx + 1] < maxp-offset):
                Distance = ((self.u0[idx] - self.u0[0])**2 + 
                                (self.u0[idx + 1] - self.u0[1])**2)
                Distance = np.sqrt(Distance)
                if Distance < maxDist:            
                    self.ClosestIdx = idx
                    maxDist = Distance
        
        print('approaching:' ,self.u0[self.ClosestIdx], self.u0[self.ClosestIdx + 1])
            
    
if __name__ == '__main__':
    slam = EKFSLAM()
    slam.AssociateLandmark(10, 50)
    
    slam.AssociateLandmark(8, 64)
    slam.AssociateLandmark(8.1, 64)
    slam.AssociateLandmark(9,65)
    slam.AssociateLandmark(80,65)
    
    
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



