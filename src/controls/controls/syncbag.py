#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
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
import pickle
import pandas as pd
import tkinter as tk
# import tkinter.messagebox as msgb

def showMessage(message,title='Info', type='info', timeout=2500):
    import tkinter as tk
    from tkinter import messagebox as msgb

    root = tk.Tk()
    root.withdraw()
    try:
        root.after(timeout, root.destroy)
        if type == 'info':
            msgb.showinfo(title, message, master=root)
        elif type == 'warning':
            msgb.showwarning('Warning', message, master=root)
        elif type == 'error':
            msgb.showerror('Error', message, master=root)
    except:
        pass
# root = tk.Tk()
# root.title("Main Window")
# def show_warning():
#     root = tk.Tk()
#     warning_window = tk.Toplevel(root)
#     warning_window.title("Warning")

#     label = tk.Label(warning_window, text="This is a warning message!")
#     label.pack(padx=10, pady=10)
#     # warning_window.destroy()

    # Close the warning window after 3000 milliseconds (3 seconds)
    # warning_window.after(3000, lambda: warning_window.destroy())
        
# field_names = ['timestamp','actiondata','latitude','longitude','orientation','angular_vel']
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
  
      self.subscriptionbutton = self.create_subscription(
            Int16,
            '/record',
            self.buttoncallback,
            10)
      self.subscriptionbutton
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
      roll = int(roll *(180/np.pi))
      pitch = int(pitch *(180/np.pi))
      yaw = int(yaw *(180/np.pi))
      self.currtime = act.header.stamp.nanosec
      self.acdata = {'timestamp':self.currtime,'actiondata':[act.data[0],act.data[1],act.data[2],act.data[3]],'latitude':gps.latitude,'longitude':gps.longitude,'roll':roll,'pitch':pitch,'yaw':yaw,'angular_velx':imu.angular_velocity.x,'angular_vely':imu.angular_velocity.y,'angular_velz':imu.angular_velocity.z}
      cv_img_farL = bridge.imgmsg_to_cv2(image_farL, desired_encoding="passthrough")
      cv_img_frontL = bridge.imgmsg_to_cv2(image_frontL, desired_encoding="passthrough")
      cv_img_frontR = bridge.imgmsg_to_cv2(image_frontR, desired_encoding="passthrough")
      cv_img_farL = cv2.cvtColor(cv_img_farL, cv2.COLOR_BGR2RGB) 
      cv_img_frontL = cv2.cvtColor(cv_img_frontL, cv2.COLOR_BGR2RGB)
      cv_img_frontR = cv2.cvtColor(cv_img_frontR, cv2.COLOR_BGR2RGB)
      self.cv_img_farL = cv2.resize(cv_img_farL,(640,640),interpolation = cv2.INTER_CUBIC)
      self.cv_img_frontL = cv2.resize(cv_img_frontL,(640,640),interpolation = cv2.INTER_CUBIC)
      self.cv_img_frontR = cv2.resize(cv_img_frontR,(640,640),interpolation = cv2.INTER_CUBIC)

      grayfarL = cv2.cvtColor(cv_img_farL, cv2.COLOR_BGR2GRAY)
      ret,self.threshfarL = cv2.threshold(grayfarL,125,255,0)

      grayfrontL = cv2.cvtColor(cv_img_frontL, cv2.COLOR_BGR2GRAY)
      ret,self.threshfrontL = cv2.threshold(grayfrontL,125,255,0)
    
      grayfrontR = cv2.cvtColor(cv_img_frontR, cv2.COLOR_BGR2GRAY)
      ret,self.threshfrontR = cv2.threshold(grayfrontR,125,255,0)
      
         
    # path = os.path.join(self.main_directory, *subfolders)
    # os.makedirs(path)
    #   self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    #   self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
      if self.record == 1 and recflag ==1:
        if len(pd.read_csv(self.csv_filename))<600:
            ctime = self.currtime
            cv2.imwrite(f'{self.main_directory}/far_left_gray/farL_{self.currtime}.png',self.threshfarL)
            cv2.imwrite(f'{self.main_directory}/front_left_gray/frontL_{self.currtime}.png',self.threshfrontL)
            cv2.imwrite(f'{self.main_directory}/front_right_gray/frontR_{self.currtime}.png',self.threshfrontR)
                
            cv2.imwrite(f'{self.main_directory}/far_left_imgs/farL_{self.currtime}.png',self.cv_img_farL)
            cv2.imwrite(f'{self.main_directory}/front_left_imgs/frontL_{self.currtime}.png',self.cv_img_frontL)
            cv2.imwrite(f'{self.main_directory}/front_right_imgs/frontR_{self.currtime}.png',self.cv_img_frontR)
            append_dict_to_csv(self.csv_filename,self.acdata)
            print('Data saved ..')
        else:
            print("Recording Ended .....")
            showMessage("Ended Recording",title="Ended Recording")
            print(f"Data saved completely to {self.main_directory}")
            self.main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
            os.makedirs(self.main_directory, exist_ok=True)
            subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs','far_left_gray','front_left_gray','front_right_gray']
            for subfolder in subfolders:
                subfolder_path = os.path.join(self.main_directory, subfolder)
                os.makedirs(subfolder_path)
            self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
            self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
            with open(self.csv_filename, 'a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                # Check if the file is empty, write header if needed
                if csv_file.tell() == 0:
                    fieldnames = ['timestamp','actiondata','latitude','longitude','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
                    csv_writer.writerow(fieldnames)
            print("New Directory created for next data")
            print("Waiting for command...")
      
          
    #   print(self.act.data)
      # print(image.width)
      # pass
      # Solve all of perception here...
   def buttoncallback(self,msg):
      
      self.record = msg.data

      if self.record==1 :
        
          showMessage("Started Recording",title="Started Recording")
        #   root = tk.Tk()
        #   root.title("Main Window")
        #   show_warning()
        #   warning_button = tk.Button(root, text="Started Recording ...", command=show_warning)
        #   warning_button.pack(pady=20)
        #   root.mainloop()
          print("Started Recording ...")
          while len(pd.read_csv(self.csv_filename))<600:
            self.save_data()
            print(f"Saving data to {self.filename}")
            print(f"{len(pd.read_csv(self.csv_filename))}")
          print("Recording Ended .....")
          showMessage("Ended Recording",title="Ended Recording")
          print(f"Data saved completely to {self.main_directory}")
          self.main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
          os.makedirs(self.main_directory, exist_ok=True)
          subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs','far_left_gray','front_left_gray','front_right_gray']
          for subfolder in subfolders:
            subfolder_path = os.path.join(self.main_directory, subfolder)
            os.makedirs(subfolder_path)
          self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
          self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
          with open(self.csv_filename, 'a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            # Check if the file is empty, write header if needed
            if csv_file.tell() == 0:
                fieldnames = ['timestamp','actiondata','latitude','longitude','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
                csv_writer.writerow(fieldnames)
          print("New Directory created for next data")
          print("Waiting for command...")


   
def main(args=None):
    
# Run the main loop
    
    rclpy.init(args=args)
    

    sb = SyncBag()
    sb.main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
    os.makedirs(sb.main_directory, exist_ok=True)
    subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs','far_left_gray','front_left_gray','front_right_gray']
    for subfolder in subfolders:
        subfolder_path = os.path.join(sb.main_directory, subfolder)
        os.makedirs(subfolder_path)
    sb.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    sb.csv_filename = f'{sb.main_directory}/data_{sb.filename}.csv'
    with open(sb.csv_filename, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        # Check if the file is empty, write header if needed
        if csv_file.tell() == 0:
            fieldnames = ['timestamp','actiondata','latitude','longitude','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
            csv_writer.writerow(fieldnames)
    
    rclpy.spin(sb)


    sb.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    