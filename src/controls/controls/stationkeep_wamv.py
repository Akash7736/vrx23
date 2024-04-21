#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import numpy as np
import time
import math
import yaml
import pyproj
from tf_transformations import euler_from_quaternion
# import tf2 as tf
with open('pid.yml', 'r') as file:
    config = yaml.safe_load(file)
# Check TA matrix , kp and kd matrices, rot matrix , dt, error rates
goal = [-33.7226643,150.673947,-1]  #-33.7226643  , 150.673947
origin = [-33.72277565710174,150.673991252778]


class PDController(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_lf = self.create_publisher(Float64, '/wamv/thrusters/leftfore/thrust', 10)
        self.publisher_rf = self.create_publisher(Float64, '/wamv/thrusters/rightfore/thrust', 10)
        self.publisher_la = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        self.publisher_ra = self.create_publisher(Float64, '/wamv/thrusters/rightaft/thrust', 10)
        self.yawpub = self.create_publisher(Float64, '/mytopic/imu/yaw', 10)
        self.cordpub = self.create_publisher(Float64, '/mytopic/geo/cord', 10)

        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning

        self.kp11 =  config['Controller']['kp11']
        self.kp22 =  config['Controller']['kp22']
        self.kp33 =  config['Controller']['kp33']
        self.kd11 =  config['Controller']['kd11']
        self.kd22 =  config['Controller']['kd22']
        self.kd33 =  config['Controller']['kd33']
        self.lex = 0
        self.ley = 0
        self.leyaw = 0
        self.l_time = time.time()
        self.c_time = None
        self.ex = 0
        self.ey = 0
        self.eyaw = 0
        self.xp = 0
        self.yp = 0
        self.yaw = 0
   
        self.TA_matrix = np.array([[0.707, 0.707, 0.707, 0.707],
                                    [-0.707, 0.707, 0.707, -0.707],
                                    [-1.626,  1.626,  -2.333,  2.333]])
        self.pseudo_TA_inv = np.linalg.pinv(self.TA_matrix)
        
        self.kp_mat = np.array([[self.kp11, 0, 0],
                                [0,self.kp22, 0],
                                [0 , 0 , self.kp33]])
        
        self.kd_mat = np.array([[self.kd11, 0, 0],
                                [0,self.kd22, 0],
                                [0 , 0 , self.kd33]])
        
        self.rot_mat = None

        self.errormat = None
        self.errorrate = None 
        self.f_local = None
        self.f_thrust = None
        self.f_global = None
        self.rot_mat = None    
        self.error_rate = None  
        self.dt = 0.1 
        self.xg,self.yg = self.gps2enu(origin[0],origin[1],goal[0],goal[1])                         
    
    
    def update(self,*args): # ex, ey ,eyaw ,yawdot 
        self.errormat = self.rot_mat@np.array([[args[0]],[args[1]],[args[2]]])  # global errors to local errors 
        self.c_time = time.time()
        # if not self.c_time == None:self.dt = self.c_time - self.l_time
        self.l_time = self.c_time
        self.error_rate = self.rot_mat@np.array([[(self.lex-args[0])/self.dt],[(self.ley-args[1])/self.dt],[args[3]]]) 
        self.lex ,self.ley,self.leyaw = args[0],args[1],args[2]
        self.f_local = self.kp_mat@self.errormat - self.kd_mat@self.error_rate
        self.f_thrust = self.pseudo_TA_inv@self.f_local
        print(self.f_thrust)
        print(f"ex: {self.ex}, ey: {self.ey}, eyaw: {self.eyaw}")
        lfmsg = Float64()
        rfmsg = Float64()
        lamsg = Float64()
        ramsg = Float64()
        lfmsg.data = float(self.f_thrust[0])
        rfmsg.data = float(self.f_thrust[1])
        lamsg.data = float(self.f_thrust[2])
        ramsg.data = float(self.f_thrust[3])
        self.publisher_lf.publish(lfmsg)
        self.publisher_rf.publish(rfmsg)
        self.publisher_la.publish(lamsg)
        self.publisher_ra.publish(ramsg)
        


    def gps2enu(self,origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
    # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        y = adjacent = math.cos(azimuth) * distance
        x = opposite = math.sin(azimuth) * distance
        return x, y
        
    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    
    def gps_callback(self,data):
       
        geo_lat = float(data.latitude)  
        geo_lon = float(data.longitude)

        self.xp,self.yp = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
        
        cord = [self.xp,self.yp]
        # geomsg = Float32MultiArray(data = cord)
        # self.cordpub.publish(geomsg)
        self.ex = self.xg - self.xp
        self.ey = self.yg - self.yp

    def imu_callback(self,data):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw = euler[2] # yaw
        self.rot_mat = np.array([[-np.sin(self.yaw), 0, 0],
                        [0, np.cos(self.yaw), 0],
                        [0 , 0 , 1]])
# yawmsg = Float64(self.yaw)
        # self.yawpub.publish(yawmsg)  # Publish yaw in radians
        self.eyaw = goal[2] - self.yaw 
        self.update(self.ex,self.ey,self.eyaw,data.angular_velocity.z)

def main(args=None):
    rclpy.init(args=args)

    skp = PDController()


    rclpy.spin(skp)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()