#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from mymessages.msg import ActionMesg
from tf_transformations import euler_from_quaternion
import cv2
import os
from cv_bridge import CvBridge
import csv 
import numpy as np
from tqdm import tqdm
bridge = CvBridge()
from datetime import datetime

main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
os.makedirs(main_directory, exist_ok=True)
subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs']
for subfolder in subfolders:
    subfolder_path = os.path.join(main_directory, subfolder)
    os.makedirs(subfolder_path)
# path = os.path.join(main_directory, *subfolders)
# os.makedirs(path)
filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
csv_filename = f'{main_directory}/data_{filename}.csv'
field_names = ['timestamp','actiondata','latitude','longitude','orientation','angular_vel']
def append_dict_to_csv(csv_filename, acdata):
    with open(csv_filename, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)

        # Check if the file is empty, write header if needed
        if csv_file.tell() == 0:
            fieldnames = list(acdata.keys())
            csv_writer.writerow(fieldnames)

        # Write the values of the dictionary as a row
        csv_writer.writerow(acdata.values())

class SyncBag(Node):
   def __init__(self):
      super().__init__('syncbag')
      # self.pub = self.create_publisher(ActionMesg,'/newtop',10)
      # acmsg = ActionMesg()
      # acmsg.data = [1.0,2.0,3.0]
      # self.pub.publish(acmsg)
      self.image_farL = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/far_left_camera_sensor/optical/image_raw')
      self.image_frontL = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw')
      self.image_frontR = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw')
      self.gps_sub = message_filters.Subscriber(self,NavSatFix,'/wamv/sensors/gps/gps/fix')
      self.imu_sub = message_filters.Subscriber(self,Imu,'/wamv/sensors/imu/imu/data')
      self.act_sub = message_filters.Subscriber(self,ActionMesg,'/actionms')
      self.ts = message_filters.ApproximateTimeSynchronizer([self.image_farL,self.image_frontL,self.image_frontR, self.act_sub,self.gps_sub,self.imu_sub], 10,0.1)
      self.ts.registerCallback(self.callback)

   def callback(self,image_farL,image_frontL,image_frontR, act,gps,imu:Imu):
    #   print(imu.orientation.x)
      roll,pitch,yaw = euler_from_quaternion([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
      currtime = act.header.stamp.nanosec
      acdata = {'timestamp':currtime,'actiondata':[act.data[0],act.data[1],act.data[2],act.data[3]],'latitude':gps.latitude,'longitude':gps.longitude,'roll':roll,'pitch':pitch,'yaw':yaw,'angular_velx':imu.angular_velocity.x,'angular_vely':imu.angular_velocity.y,'angular_velz':imu.angular_velocity.z}
      cv_img_farL = bridge.imgmsg_to_cv2(image_farL, desired_encoding="passthrough")
      cv_img_frontL = bridge.imgmsg_to_cv2(image_frontL, desired_encoding="passthrough")
      cv_img_frontR = bridge.imgmsg_to_cv2(image_frontR, desired_encoding="passthrough")
      cv2.imwrite(f'{main_directory}/far_left_imgs/farL_{currtime}.png',cv_img_farL)
      cv2.imwrite(f'{main_directory}/front_left_imgs/frontL_{currtime}.png',cv_img_frontL)
      cv2.imwrite(f'{main_directory}/front_right_imgs/frontR_{currtime}.png',cv_img_frontR)
      append_dict_to_csv(csv_filename,acdata)
      print('Data saved ..')
      print(act.data)
      # print(image.width)
      # pass
      # Solve all of perception here...
   
def main(args=None):
    rclpy.init(args=args)

    sb = SyncBag()


    rclpy.spin(sb)

    sb.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()