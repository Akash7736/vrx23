import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import time
import math
      




class PDController:

    def __init__(self,kp,kd):
        self.kp = kp 
        self.kd = kd 
        self.lasterror = 0
        self.last_time = 0
        self.TA_matrix = np.array([[0.707, -0.707, -0.707, 0.707],
                                    [0.707, 0.707, 0.707, 0.707],
                                    [0.35,  0.35,  0.35,  0.35]])
                                   
        self.pseudo_TA_inv = np.linalg.pinv(self.TA_matrix)
        self.parama = np.array([ 0.01, 59.82,5, 0.38, 0.56, 0.28])
        
        self.paramb = np.array([ -199.13, -0.09, 8.84, 5.34, 0.99, -0.57])
   
        

    def update(self,error):
        present_time = time.time()
        dt = present_time - self.last_time # change in time
        dt = 0.1
        derivative = (error- self.lasterror)/dt # rate of change of error
        self.lasterror = error
        self.last_time = present_time

        return (self.kp * error) + (self.kd * derivative)
    
    def inv_fin_func(self,x):

        if(x>=0):
            A = self.parama[0]
            K = self.parama[1]
            B = self.parama[2]
            v = self.parama[3]
            C = self.parama[4]
            M = self.parama[5]
            pwmfrac = ( (-1/B) * math.log((((K-A)/(x-A))**v)-C) ) + M
            return pwmfrac*100
        elif(x<0):
            A = self.paramb[0]
            K = self.paramb[1]
            B = self.paramb[2]
            v = self.paramb[3]
            C = self.paramb[4]
            M = self.paramb[5]
            pwmfrac = ( (-1/B) * math.log((((K-A)/(x-A))**v)-C) ) + M
            return pwmfrac*100

# pd = PDController(0.1,0.2)
# print(pd.inv_fin_func(50))