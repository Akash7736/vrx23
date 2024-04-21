#!/usr/bin/env python3
import rclpy
# Final output is pwm
from rclpy import Node
import numpy as np
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
# from pyproj import Geod
import tf

import math
import time

# g = Geod(ellps ='WGS84')

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
        self.parama1 = np.array([ 1.52503318e-05, -4.91422601e-02,  3.93300223e+01])
        self.parama2 = np.array([ 1.61019834e-05, -4.23940889e-02,  2.71121120e+01])
        self.paramb1 = np.array([ 1.67759057e-05, -5.46903629e-02,  4.41751206e+01])
        self.paramb2 = np.array([ 2.12621123e-05, -5.82516683e-02,  3.93836637e+01])
        


    def update(self,error):
        present_time = time.time()
        dt = present_time - self.last_time # change in time
        dt = 0.1
        derivative = (error- self.lasterror)/dt # rate of change of error
        self.lasterror = error
        self.last_time = present_time

        return (self.kp * error) + (self.kd * derivative)
    

         
# 
        # error_rate = np.array([[u],
        #                     [v],
        #                     [r]])
        # error_matrix = np.array([[error_x],
        #                         [error_y],
        #                         [error_yaw]])
        # F_matrix = np.dot(kp,error_matrix) + np.dot(kd,error_rate)
        # thrust_out = np.dot(pseudo_TA_inv,F_matrix)
        # for i in thrust_out:
        #     pwm_out.append(inv_fin_func(i))
        #     print(pwm_out)
        # return pwm_out
    def fin_curve_l(self,t):
        return (0.4*(self.parama1[0]*(t**2) + self.parama1[1]*t +  self.parama1[2])) + (0.6*(self.paramb1[0]*(t**2) + self.paramb1[1]*t + self.paramb1[2]))

    def fin_curve_r(self,t):
        return (0.4*(self.parama2[0]*(t**2) + self.parama2[1]*t +  self.parama2[2])) + (0.6*(self.paramb2[0]*(t**2) + self.paramb2[1]*t + self.paramb2[2]))
    
    def fin_func(self,x):
        if 1100<=x<1470:
            return self.fin_curve_l(x)
        elif 1470<=x<=1530:
            return 0
        elif 1530<x<=1900:
            return self.fin_curve_r(x)
    

    def equationroots(self, x, y, z): 

        discri = y * y - 4 * x * z
        sqrtval = math.sqrt(abs(discri)) 
        # checking condition for discriminant
        if discri > 0: 
            return [(-y + sqrtval)/(2 * x), (-y - sqrtval)/(2 * x)]
        elif discri == 0: 
            return [(-y / (2 * x))]
        else:
            return [1500]
        

    def inv_fin_func(self,x):
        
        a = 0.4*self.parama1[0] + 0.6*self.paramb1[0]
        b = 0.4*self.parama1[1] + 0.6*self.paramb1[1]
        c = 0.4*self.parama1[2] + 0.6*self.paramb1[2]
        s1 = self.equationroots(-a,-b,-c-x)
        s2 = self.equationroots((0.4*self.parama2[0] + 0.6*self.paramb2[0]),(0.4*self.parama2[1]+0.6*self.paramb2[1]) ,(0.4*self.parama2[2] + 0.6*self.paramb2[2]-x))
        if (x>0 and x<0.5) :  ## Forward
            for i in range(0,len(s2)):
                if s2[i]>1500:
                    pwm = s2[i]
        elif (-0.5<x<0):  ### Reverse
            for i in range(0,len(s1)):
                if s1[i]<1500:
                    pwm = s1[i]
        elif x<=-0.5:
            pwm = 1400
        elif x>=0.5:
            pwm = 1600
        else:
            pwm = 1500
        return pwm
        # if x>4.079:
        #     for i in range(0,len(s2)):
        #         # print(s[i])
        #         if 1500<s2[i]<=1900:
        #             pwm[0] = s2[i]
        #             pwm[0] = 1500
        # elif 0<x<4.079:
        #     for i in range(0,len(s2)):
        #         # print(s[i])
        #         if 1500<s2[i]<=1900:
        #             pwm[0] = s2[i]
        # # return s
        #     for i in range(0,len(s1)):
        #         # print(s[i])
        #         if 0<s1[i]<=1500:
        #             pwm[0] = s1[i]
        # elif x == 0:
        #     pwm[0] = 1500
        #     pwm[0] = 1500
        # if x<0:
        #     return 3000 - pwm[0]
        # else:
        #     return pwm[0]
  

# PDC = PDController(0.3,0.6)

# pwm_out = []
# n = 4 # number of waypoints 


# TA_inv = np.linalg.inv(TA_matrix)



# kd = np.array([[PDC.kd, 0, 0],
#                [0, PDC.kd, 0],
#                [0, 0, PDC.kd]])





# origin = [] 
# origin_flag = 0

# def gps_callback(data):
#     #rospy.loginfo("Received GPS data: Latitude: %f, Longitude: %f", data.latitude, data.longitude)
#     global my_lat
#     my_lat = data.latitude  # lat and long to convert
#     global my_lon
#     my_lon = data.longitude
#     current_state =[my_lat,my_lon]
#     if origin_flag==0:
#         origin.append(my_lat,my_lon)
#         origin_flag = 1
#     wp_x_y = g.inv(origin[0],origin[1],waypoint_cord[0][0],waypoint_cord[0][1])
#     wp_x = wp_x_y[0]
#     wp_y = wp_x_y[1]
#     cord = g.inv(origin[0], origin[1], my_lon, my_lat)
#     global x 
#     x = cord[0]
#     global y 
#     y = cord[1]
#     global error_x
#     error_x = wp_x - x
#     global error_y
#     error_y = wp_y - y
#############################################  calculating  pwm from thrust 





###################################################

# def imu_callback(data):
#     quaternion = data
#     euler = tf.transformations.euler_from_quaternion(quaternion) # quaterion to euler angle conversion
#     global yaw 
#     yaw = euler[2] # roll, pitch , yaw
#     #rospy.loginfo("Received IMU data: Yaw: %f", yaw)
#     global error_yaw
#     error_yaw = desired_yaw - yaw
#     PDC.update()

##################################################################

# def tracker():
#     rospy.init_node('tracker',anonymous=True)
#     rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_callback)
#     rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)


# if __name__ == '__main__':
#     tracker()
#     rospy.spin()
