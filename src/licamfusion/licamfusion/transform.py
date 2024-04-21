#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node  
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from std_msgs.msg import Header
import numpy as np
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def rot_x(theta):
    R = np.array([[1,0,0],
         [0,np.cos(theta),-np.sin(theta)],
         [0,np.sin(theta),np.cos(theta)]])
    return R

def rot_y(theta):
    R = np.array([[np.cos(theta),0,np.sin(theta)],
         [0,1,0],
         [-np.sin(theta),0,np.cos(theta)]])
    return R

def rot_z(theta):
    R = np.array([[np.cos(theta),-np.sin(theta),0],
         [np.sin(theta),np.cos(theta),0],
          [0,0,1],])
    return R



class Transform(Node):

    def __init__(self):
        super().__init__('transformer')
        self.subscriptiontf = self.create_subscription(TFMessage,'/tf',self.tf_callback,10)
        self.subscriptiontf  # prevent unused variable warning

    def dummy(self,data:TransformStamped):
        data.transform
        pass
  

    def tf_callback(self,data:TFMessage):
        # print(len(data._transforms))
        # print(type(data._transforms[0]))
        tfdict = {}
        
        for i in range(0,len(data._transforms)): tfdict[data._transforms[i].header.frame_id +' to ' + data._transforms[i].child_frame_id] = data._transforms[i].transform
        # print(tfdict['wamv/wamv/imu_wamv_link to wamv/wamv/imu_wamv_link/imu_wamv_sensor'])
        print(tfdict)
        # data._transforms[0].


def main(args=None):
    rclpy.init(args=args)

    tfm = Transform()


    rclpy.spin(tfm)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()