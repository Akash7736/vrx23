import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import numpy as np
import pyproj
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
import sensor_msgs_py.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2 
import pyproj
from cv_bridge import CvBridge
bridge = CvBridge()
import matplotlib.pyplot as plt
import pickle
from datetime import datetime
import os
import csv
import message_filters
from mymessages.msg import ActionMesg
from std_msgs.msg import Int16
import tkinter as tk
import pandas as pd
import time
import matplotlib as mpl

lidar_link = 'wamv/wamv/lidar_wamv_link'
camera_link =  'wamv/wamv/front_left_camera_link'
base_link = 'wamv/wamv/base_link'
imu_link = 'wamv/wamv/imu_wamv_link'
worldlink = 'map'
origin = [-33.72277565710174,150.673991252778]

def count_files(directory):
    if not os.path.isdir(directory):
        print(f"Error: {directory} is not a directory.")
        return

    file_count = 0
    for _, _, files in os.walk(directory):
        file_count += len(files)
    
    return file_count

def append_dict_to_csv(csv_filename, acdata):
    with open(csv_filename, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)

        # Check if the file is empty, write header if needed
        if csv_file.tell() == 0:
            fieldnames = list(acdata.keys())
            csv_writer.writerow(fieldnames)

        # Write the values of the dictionary as a row
        csv_writer.writerow(acdata.values())

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

class Imidata:
    def __init__(self,timestamp,action,grid=None,latitude=None,longitude = None, yaw = None, x = None, y = None):
        self.timestamp = timestamp
        self.grid = grid
        self.action = action
        self.latitude = latitude
        self.longitude = longitude 
        self.yaw = yaw
        self.x = x
        self.y = y

class FrameListener(Node):

    def __init__(self):
        super().__init__('Licamfuser')

        self.xp = 0.0
        self.yp = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.clocktime  = 0
        self.heightthresh = 1
        self.cv_image = None
        self.recflag = False
        self.c = 0
        self.mc = 0
        self.msgflag = 1
        self.savecount = 0
        self.action = [0,0,0,0]
        self.cam_matrix = np.array([[762.7223205566406, 0.0, 640.0],
                                    [0.0, 762.7223110198975, 360.0],
                                    [0.0, 0.0, 1.0]]) 
        self.K = np.array([762.7223205566406, 0.0, 640.0, 0.0,
                          762.7223110198975, 360.0, 0.0, 0.0, 1.0]).reshape((3, 3))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tfb_ = TransformBroadcaster(self)
        self.filpublisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        # self.subscriptionvoxel = self.create_subscription(PointCloud2,'/myvoxel',self.voxel_callback,10)
        # self.subscriptionvoxel 
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning
        # self.subscriptionimg = self.create_subscription(Image,'/wamv/sensors/cameras/far_left_camera_sensor/image_raw',self.image_callback,10)
        # self.subscriptionimg 
        self.subscriptionbutton = self.create_subscription(
                Int16,
                '/record',
                self.buttoncallback,
                10)
        self.subscriptionbutton
        # self.image_farL = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/far_left_camera_sensor/optical/image_raw')
        # self.image_frontL = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw')
        # self.image_frontR = message_filters.Subscriber(self,Image,'/wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw')
        # self.voxelsub = message_filters.Subscriber(self,PointCloud2,'/filtered_points')
        self.gps_sub = message_filters.Subscriber(self,NavSatFix,'/wamv/sensors/gps/gps/fix')
        self.imu_sub = message_filters.Subscriber(self,Imu,'/wamv/sensors/imu/imu/data')
        self.act_sub = message_filters.Subscriber(self,ActionMesg,'/actionms')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.act_sub,self.gps_sub,self.imu_sub], 10,0.1)
        self.ts.registerCallback(self.callback)
    
    def gps2enu(self,origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
    # Convert azimuth to radians
        azimuth = np.radians(azimuth)
        y = adjacent = np.cos(azimuth) * distance
        x = opposite = np.sin(azimuth) * distance
        return x, y
 
    def callback(self, act,gps:NavSatFix,imu:Imu):
        print('Callback started')
        self.roll_rad,self.pitch_rad,self.yaw_rad = euler_from_quaternion([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        self.roll = round((self.roll_rad *(180/np.pi)),2)
        self.pitch = round((self.pitch_rad *(180/np.pi)),2)
        self.yaw = round((self.yaw_rad *(180/np.pi)),2)
        geo_lat = float(gps.latitude)  
        geo_lon = float(gps.longitude)
        self.x2,self.y2 = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
        self.x2,self.y2 = round(self.x2,2),round(self.y2,2) 


        self.latitude = gps.latitude
        self.longitude = gps.longitude
        self.currtime = act.header.stamp.nanosec
        self.action = [act.data[0],act.data[1],act.data[2],act.data[3]]
        # self.msgflag = 0
        # self.acdata = {'timestamp':self.currtime,'actiondata':[act.data[0],act.data[1],act.data[2],act.data[3]],'latitude':gps.latitude,'longitude':gps.longitude,'roll':roll,'pitch':pitch,'yaw':yaw,'angular_velx':imu.angular_velocity.x,'angular_vely':imu.angular_velocity.y,'angular_velz':imu.angular_velocity.z}
    
        if self.recflag==True:
            print(len(pd.read_csv(self.csv_filename)))
            
            if len(pd.read_csv(self.csv_filename))<600:
                if self.msgflag == 1:
                    showMessage("Started Recording",title="Started Recording",timeout=500)
                    self.msgflag = 0
                
                # imidata = Imidata(self.currtime,self.action,yaw=self.yaw,x=self.xp,y=self.yp)
                # file_name = f"{self.main_directory}/pickle_files/data_{self.currtime}.pkl"
                # with open(file_name, 'wb') as f:
                #     pickle.dump(imidata, f)
                # ctime = self.currtime
                # time.sleep(3)
                # self.voxel_callback(voxelmsg)
                # cv2.imwrite(f'{self.main_directory}/far_left_gray/farL_{self.currtime}.png',self.threshfarL)
                # cv2.imwrite(f'{self.main_directory}/front_left_gray/frontL_{self.currtime}.png',self.threshfrontL)
                # cv2.imwrite(f'{self.main_directory}/front_right_gray/frontR_{self.currtime}.png',self.threshfrontR)
                    
                # cv2.imwrite(f'{self.main_directory}/far_left_imgs/farL_{self.currtime}.png',self.cv_img_farL)
                # cv2.imwrite(f'{self.main_directory}/front_left_imgs/frontL_{self.currtime}.png',self.cv_img_frontL)
                # cv2.imwrite(f'{self.main_directory}/front_right_imgs/frontR_{self.currtime}.png',self.cv_img_frontR)
                self.acdata = {'timestamp':self.currtime,'actiondata':self.action,'latitude':gps.latitude,'longitude':gps.longitude,'xp':self.x2,'yp':self.y2,'roll':self.roll,'pitch':self.pitch,'yaw':self.yaw,'angular_velx':imu.angular_velocity.x,'angular_vely':imu.angular_velocity.y,'angular_velz':imu.angular_velocity.z}

                append_dict_to_csv(self.csv_filename,self.acdata)
                plt.xlabel('X')
                plt.ylabel('Y')
                plt.title('Scatter Plot of Points')
                # 70.38,45.43
                # plt.scatter(70.38,45.43,color='r')
                # plt.scatter(int(self.x2), int(self.y2),color='b',s=2)
                self.mc+=1
                arrow = u'$\u2191$'
                rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
                rotated_marker._transform = rotated_marker.get_transform().rotate_deg(self.yaw)
                if self.mc%10==0:plt.scatter((int(self.x2)), int((self.y2)), marker=rotated_marker, s=100, facecolors='none', edgecolors='b')
                arrow_length = 0.5
                arrow = u'$\u2191$'

                # for i, val in enumerate([0,90,180,270,360]):
                rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
                rotated_marker._transform = rotated_marker.get_transform().rotate_deg(52.26)
                plt.scatter((70.38), (45.43), marker=rotated_marker, s=150, facecolors='none', edgecolors='r')

                # plt.quiver(70.38,45.43, arrow_length*np.cos(self.yaw_rad+np.pi/2),arrow_length*np.sin(self.yaw_rad+np.pi/2), angles='xy', scale_units='xy', scale=1, color='green', width=0.02, headwidth=4) # working

                # plt.quiver(self.x2 , self.y2, arrow_length*np.cos(self.yaw_rad+np.pi/2),arrow_length*np.sin(self.yaw_rad+np.pi/2), angles='xy', scale_units='xy', scale=1, color='green', width=0.02, headwidth=4) # working


                print('Data saved ..')
            else:
                print("Recording Ended .....")
                self.msgflag = 1
                plt.savefig(f"{self.main_directory}/maps/grid_{self.currtime}.png")
                plt.cla()
                # self.recflag = False
                self.c+=1
                showMessage("Length max Ended Recording",title="Length max Ended Recording")
                print(f"Data saved completely to {self.main_directory}")
                self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
                self.main_directory = f'IMLdataset/dataset_{self.filename}'
                os.makedirs(self.main_directory, exist_ok=True)
                subfolders = ['pickle_files','maps']
                for subfolder in subfolders:
                    subfolder_path = os.path.join(self.main_directory, subfolder)
                    os.makedirs(subfolder_path)
                # self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
                self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
                with open(self.csv_filename, 'a', newline='') as csv_file:
                    csv_writer = csv.writer(csv_file)
                # Check if the file is empty, write header if needed
                    if csv_file.tell() == 0:
                        fieldnames = ['timestamp','actiondata','latitude','longitude','xp','yp','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
                        csv_writer.writerow(fieldnames)
                # self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
                # with open(self.csv_filename, 'a', newline='') as csv_file:
                #     csv_writer = csv.writer(csv_file)
                #     # Check if the file is empty, write header if needed
                #     if csv_file.tell() == 0:
                #         fieldnames = ['timestamp','actiondata','latitude','longitude','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
                #         csv_writer.writerow(fieldnames)
                print("New Directory created for next data")
                print("Waiting for command...")
    def transform_point(self, transformation, point_wrt_source):
        point_wrt_target = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_source), transformation).point
        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]
    

    def buttoncallback(self,msg):
        print(self.recflag)
        
        self.record = msg.data
        if self.record == 1: 
            # self.recflag = True
            self.c+=1
            # showMessage("Stopped Recording",title="Stopped Recording",timeout=500)
            # showMessage("Ended Recording",title="Ended Recording",timeout=500)
        if self.c%2==0:
            self.recflag = False         
            
        else: 
            self.recflag = True
            
            

        # if self.record==1 :
            
        #     showMessage("Started Recording",title="Started Recording")
        #     #   root = tk.Tk()
        #     #   root.title("Main Window")
        #     #   show_warning()
        #     #   warning_button = tk.Button(root, text="Started Recording ...", command=show_warning)
        #     #   warning_button.pack(pady=20)
        #     #   root.mainloop()
        #     print("Started Recording ...")
        #     while len(pd.read_csv(self.csv_filename))<600:
        #         # self.save_data()
        #         print(f"Saving data to {self.filename}")
        #         print(f"{len(pd.read_csv(self.csv_filename))}")
        #     print("Recording Ended .....")
        #     showMessage("Ended Recording",title="Ended Recording")
        #     print(f"Data saved completely to {self.main_directory}")
        #     self.main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
        #     os.makedirs(self.main_directory, exist_ok=True)
        #     subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs','far_left_gray','front_left_gray','front_right_gray']
        #     for subfolder in subfolders:
        #         subfolder_path = os.path.join(self.main_directory, subfolder)
        #         os.makedirs(subfolder_path)
        #     self.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        #     self.csv_filename = f'{self.main_directory}/data_{self.filename}.csv'
        #     with open(self.csv_filename, 'a', newline='') as csv_file:
        #         csv_writer = csv.writer(csv_file)
        #         # Check if the file is empty, write header if needed
        #         if csv_file.tell() == 0:
        #             fieldnames = ['timestamp','actiondata','latitude','longitude','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
        #             csv_writer.writerow(fieldnames)
        #     print("New Directory created for next data")
        #     print("Waiting for command...")



    def voxel_callback(self, msg:PointCloud2):
        self.clocktime = msg.header.stamp
  
        pt_arr = []
        pointsincam  = []
        gridset =  set()
        gridset.clear()
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            point_wrt_lidar = Point()
            point_wrt_lidar.x = float(pt_x)
            point_wrt_lidar.y = float(pt_y)
            point_wrt_lidar.z = float(pt_z)
            from_frame_rel = lidar_link
            to_frame_rel = base_link
            t1 = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time(), rclpy.duration.Duration(seconds=4.0))
            pt2 = self.transform_point(t1, point_wrt_lidar)
            point_wrt_base = Point()
            point_wrt_base.x = float(pt2[0])
            point_wrt_base.y = float(pt2[1])
            point_wrt_base.z = float(pt2[2])

            from_frame_rel = base_link
            to_frame_rel = worldlink
            t2 = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time()) 
            point_wrt_world = self.transform_point(t2, point_wrt_base)           # print([pt_x,pt_y,pt_z])
            if point_wrt_world[2]  > self.heightthresh and point_wrt_world[2] < 1.5 and pt2[0]<20.0 and pt2[1]<20.0:
                pt_arr.append([point_wrt_lidar.x,point_wrt_lidar.y,point_wrt_lidar.z])

                from_frame_rel = base_link
                to_frame_rel = camera_link
                t3 = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time()) 
                point_wrt_cam = self.transform_point(t3, point_wrt_base) 
                pointsincam.append(point_wrt_cam) 
                gridset.add((round(point_wrt_world[0],1),round(point_wrt_world[1],1)))

        x_coords, y_coords = zip(*gridset)
        X, Y = np.meshgrid(x_coords, y_coords)

# Plot the points
        plt.cla()
        plt.scatter(x_coords, y_coords)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Scatter Plot of Points')
        plt.grid(True)
        plt.xlim(-20,20)
        plt.ylim(-20,20)
        plt.savefig(f"{self.main_directory}/gridmaps/grid_{self.currtime}.png")
        imidata = Imidata(self.currtime,gridset,self.action,yaw=self.yaw,x=self.xp,y=self.yp)
        file_name = f"{self.main_directory}/pickle_files/grid_{self.currtime}.pkl"
        with open(file_name, 'wb') as f:
            pickle.dump(imidata, f)
        print(f'len: {len(gridset)}' ) 
        gridset.clear()     
        # pointsincam = np.array(pointsincam)
        # uv = self.XYZ_to_UV(self.K,pointsincam)
        # print(uv)
        # for i in uv:
        #     # print(i[0])
        #     if  self.cv_image is not None : 
        #         cv2.drawMarker(self.cv_image, (int(i[0]),int(i[1])), (0,0,255), cv2.MARKER_CROSS, 5, 2)

        #         # self.cv_image[int(i[0]),int(i[1])] = (0,0,255)
        # cv2.imshow("img",self.cv_image)
        # key = cv2.waitKey(1)

        pt_arr = np.array(pt_arr)
        header = Header()
        header.stamp = self.clocktime
        header.frame_id = base_link
        pc_msg = point_cloud2.create_cloud_xyz32(header, pt_arr)
        # Publish the PointCloud2 message
        self.filpublisher.publish(pc_msg)
        print(pt_arr.shape)
        print("Filtered Points Published ...........")
        

    def image_callback(self, msg):
    
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        print(f"image shape : {self.cv_image.shape}")
        self.cv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_BGRA2BGR)

    def XYZ_to_UV(self,K, points: np.ndarray):
        trans_points = np.zeros((len(points), 3))
        trans_points[:, 2] = points[:, 0]
        trans_points[:, 0] = points[:, 1] * -1
        trans_points[:, 1] = points[:, 2] * -1

        pixels = np.dot(K, trans_points.T).T
        pixels = np.int32(pixels[:, :2] / pixels[:, 2].reshape(-1, 1))

        return pixels

    def gps_callback(self,data:NavSatFix):
       
        geo_lat = float(data.latitude)  
        geo_lon = float(data.longitude)

        self.xp,self.yp = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
        
        cord = [self.xp,self.yp]

    def imu_callback(self,data:Imu):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw1 = euler[2] # yaw
        self.roll1 = euler[0]
        self.pitch1 = euler[1]
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id="map"
        tfs._child_frame_id = base_link
        tfs.transform.translation.x = self.xp
        tfs.transform.translation.y = self.yp
        tfs.transform.translation.z = 0.0  

        r = R.from_euler('xyz',[self.roll1,self.pitch1,self.yaw1])

        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]

        self.tfb_.sendTransform(tfs)    



def main(args=None):
    rclpy.init(args=args)

    fl = FrameListener()
    fl.main_directory = f'IMLdataset/dataset_{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}'
    os.makedirs(fl.main_directory, exist_ok=True)
    # subfolders = ['far_left_imgs', 'front_left_imgs', 'front_right_imgs','far_left_gray','front_left_gray','front_right_gray']
    subfolders = ['pickle_files','maps']
    for subfolder in subfolders:
        subfolder_path = os.path.join(fl.main_directory, subfolder)
        os.makedirs(subfolder_path)
    fl.filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    fl.csv_filename = f'{fl.main_directory}/data_{fl.filename}.csv'
    with open(fl.csv_filename, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        # Check if the file is empty, write header if needed
        if csv_file.tell() == 0:
            fieldnames = ['timestamp','actiondata','latitude','longitude','xp','yp','roll','pitch','yaw','angular_velx','angular_vely','angular_velz']
            csv_writer.writerow(fieldnames)
    

    rclpy.spin(fl)

    fl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()