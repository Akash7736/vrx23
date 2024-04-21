#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from acoskp import PDController
from ros_gz_interfaces.msg import ParamVec
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import numpy as np
import time
import math
import pyproj
import yaml
from tf_transformations import euler_from_quaternion
origin = [-33.72277565710174,150.673991252778]

class Acoustics(Node):
    def __init__(self):
        super().__init__('acoustics')
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.pingersubscription = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.ping_callback,
            10)
        self.pingersubscription
        self.publisher_goal = self.create_publisher(PoseStamped, '/vrx/stationkeeping/goal', 10)

        self.bear = 0
        self.ran = 0
        self.ele = 0

        self.xp = 0
        self.yp = 0
        self.x_l = 0
        self.y_l = 0
        self.globalcord = None
        self.rot_mat = None  


    def ping_callback(self,data: ParamVec):
        # data.params[]
        print(data.header.frame_id)
        # # print(dir(data.params[0]))
 
        di = {}
        for d in data.params:
            di[d.name] = d.value.double_value
        print(di)
        self.bear = di['bearing']
        self.ran = di['range']
        self.ele = di['elevation']
        self.x_l = self.ran*np.cos(self.ele)*np.cos(self.bear)
        self.y_l = self.ran*np.cos(self.ele)*np.sin(self.bear)
        self.globalcord = np.linalg.inv(self.rot_mat)@np.array([[self.x_l],[self.y_l],[0]]) - np.array([[self.xp],[self.yp],[0]])
        print(f"x_l:{self.x_l}  y_l:{self.y_l}")
        goalmsg = PoseStamped()
        goalmsg.pose.position.x = self.globalcord[0][0]
        goalmsg.pose.position.y = self.globalcord[1][0]
        q = quaternion_from_euler(0,0,0)
        goalmsg.pose.orientation.x,goalmsg.pose.orientation.y,goalmsg.pose.orientation.z,goalmsg.pose.orientation.w = q[0],q[1],q[2],q[3]
        self.publisher_goal.publish(goalmsg)


    def gps2enu(self,origin_lat, origin_long, goal_lat, goal_long):
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
        azimuth = math.radians(azimuth)
        y = adjacent = math.cos(azimuth) * distance
        x = opposite = math.sin(azimuth) * distance
        return x, y

    def gps_callback(self,data):
       
        geo_lat = float(data.latitude)  
        geo_lon = float(data.longitude)

        self.xp,self.yp = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
    def imu_callback(self,data):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw = euler[2] # yaw
        self.rot_mat = np.array([[-np.sin(self.yaw), 0, 0],
                        [0, np.cos(self.yaw), 0],
                        [0 , 0 , 1]])
    
 
def main(args=None):
    rclpy.init(args=args)
   
    ping = Acoustics()
    rclpy.spin(ping)
    

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()