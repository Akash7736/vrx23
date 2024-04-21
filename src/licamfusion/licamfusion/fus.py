#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node  
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from std_msgs.msg import Header
import numpy as np
from tf_transformations import euler_from_quaternion
from quattoeul import tf2tfmat
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2
import cv2 as cv 
from cv_bridge import CvBridge
bridge = CvBridge()
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
# import keyboard
# import pynput
# from pynput.keyboard import Key, Listener

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


class Fuse(Node):

    def __init__(self):
        super().__init__('tran')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.cam_matrix = np.array([[762.7223205566406, 0.0, 640.0],
                                    [0.0, 762.7223110198975, 360.0],
                                    [0.0, 0.0, 1.0]])  
        self.bltoli = np.array([[1, 0, 0, 0.7],
                                [0, 1, 0, 0],
                                [0, 0, 1, 1.8],
                                [0, 0, 0, 1]])
        self.bltocam =  np.array([[ 0.96592593, 0, 0.25881867, 0.75],
                                  [0, 1, 0, 1],
                                  [-0.25881867, 0, 0.96592593, 1.5],
                                  [0, 0, 0, 1]])
        self.cluspub = self.create_publisher(PointCloud2, '/mycluster', 10)
        self.subscriptiontf = self.create_subscription(TFMessage,'/tf',self.tf_callback,10)
        self.subscriptiontf  # prevent unused variable warning
        self.subscriptionimg = self.create_subscription(Image,'/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw',self.image_callback,10)
        self.subscriptionimg  # prevent unused variable warning
        # self.subscriptionvoxel = self.create_subscription(PointCloud2,'/wamv/sensors/lidars/lidar_wamv_sensor/points',self.voxel_callback,10)
        self.subscriptionvoxel = self.create_subscription(PointCloud2,'/myvoxel',self.voxel_callback,10)
        self.subscriptionvoxel 
        self.cv_image = None
        self.image_count = 0
        
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations

        try:
            tlis = [self.tf_buffer.lookup_transform('wamv/wamv/lidar_wamv_link','wamv/wamv/base_link', rclpy.time.Time()),
            self.tf_buffer.lookup_transform('wamv/wamv/front_left_camera_link_optical','wamv/wamv/base_link', rclpy.time.Time())]
            trans = {}
            for i in [0,1]: trans[i] = tf2tfmat(tlis[i])
        

            self.bltoli = trans[0]    #np.array([[1, 0, 0, 0.7],
                                #[0, 1, 0, 0],
                               # [0, 0, 1, 1.8],
                                #[0, 0, 0, 1]])
            self.bltocam =   trans[1]      #np.array([[ 0.96592593, 0, 0.25881867, 0.75],
                                  #[0, 1, 0, 1],
                                  #[-0.25881867, 0, 0.96592593, 1.5],
                                  #[0, 0, 0, 1]])
            # print([t.transform.translation.y,t.transform.translation.x])
            # print(self.bltoli)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform')
            return



    def dummy(self,data:Marker):
        data._points
        pass
  

    def tf_callback(self,data:TFMessage):
        # print(len(data._transforms))
        # print(type(data._transforms[0]))
        tfdict = {}
        
        for i in range(0,len(data._transforms)): tfdict[data._transforms[i].header.frame_id +' to ' + data._transforms[i].child_frame_id] = data._transforms[i].transform
        # print(tfdict)
        # print(type(tfdict['wamv/wamv/base_link to wamv/wamv/base_link/front_left_camera_sensor']))
        s ='wamv/wamv/base_link to wamv/wamv/base_link/front_left_camera_sensor'
        s1 = 'wamv/wamv/base_link to wamv/wamv/base_link/lidar_wamv_sensor'
        # if tfdict.get(s)is not None: print(f"trns1: {tfdict[s].translation}") 
        # if tfdict.get(s)is not None: print(f"rot1 : {tfdict[s].rotation}") 
        # if tfdict.get(s1)is not None: print(f"trns2: {tfdict[s1].translation}") 
        # if tfdict.get(s1)is not None: print(f"rot2 : {tfdict[s1].rotation}") 
        # data._transforms[0].

    def image_callback(self, msg):
 
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        # print(self.cv_image .shape)
        self.cv_image = cv.resize(self.cv_image, (1280,720))
        self.cv_image = cv.cvtColor(self.cv_image,cv.COLOR_BGRA2BGR)
        # (b, g, r) = self.cv_image[0, 0]
        # print(b)
# 
        

    def voxel_callback(self, msg:PointCloud2):


        xyz = np.array(list(sensor_msgs_py.point_cloud2.read_points(msg,field_names = ("x", "y", "z"), skip_nans=True)))
        xyz = [list(ele) for i,ele in enumerate(xyz)]
        # print(xyz)
     
  
        pt_arr = []
        for point in sensor_msgs_py.point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            # print([pt_x,pt_y,pt_z])
            pt_arr.append([pt_x,pt_y,pt_z])
        # print(pt_arr)

        # model = DBSCAN(eps=3, min_samples=2)
        # model.fit_predict(xyz)
        # db = DBSCAN(eps=0.5, min_samples=5).fit(xyz)
            
        db = DBSCAN(eps=0.1, min_samples=5)
        # pred = model.fit_predict(np.array(pt_arr))
        # clustering = DBSCAN(eps=7, min_samples=2).fit(np.array(pt_arr))
        # print(model.labels_,len(model.labels_),model.labels_.max())
        labels = db.fit_predict(pt_arr)
        print(labels)
        unique_labels = np.unique(labels[labels != -1])

        # Calculate centroid for each cluster
        # for label in unique_labels:
            # cluster_points = pt_arr[(labels == label).astype(bool)]
            # centroid = np.mean(cluster_points, axis=0)
            # print(f"Cluster {label} - Centroid: {centroid}")
            # print(label.shape)

        labels = db.labels_
        no_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        # print(no_clusters)
        clus_centre = []
        for i in range(0,no_clusters):
            clus_centre.append(np.mean(np.array(pt_arr)[labels==i,:], axis=0))
        # print(clus_centre)
        # points_of_cluster_0 = np.array(pt_arr)[labels==0,:]
        centroid_of_cluster_0 = np.mean(np.array(pt_arr)[labels==0,:], axis=0) 
        clus_centre = np.array(clus_centre)
        # print(clus_centre)
        K = np.reshape(np.array([762.7249337622711, 0.0, 640.5, 0.0, 762.7249337622711, 360.5, 0.0, 0.0, 1.0]),(3,3))
        for i in clus_centre:
            dis = np.linalg.norm(i)
            if dis<10:
                # tvec x=-0.4426760663909814, y=0.0, z=-1.8799036944068728)
                # rvec 0, 0.1391689, 0
                # rvec 1.40,-1.36,1.05
                # tvec -0.25, -0.49, -0.14
                imagePoint , _ = cv.projectPoints(i, np.array([[-1.6929693744344996, 2.7755575615628914e-16, -1.5707963267948963 ]], dtype=float), np.array([0.0912,0.1,-0.290], dtype=float), K,np.array([[0,0,0,0,0]], dtype=float))
                imagePoint = np.squeeze(imagePoint,1)
                if 0<imagePoint[0][0]<1280 and 0<imagePoint[0][1]<720 and self.cv_image is not None : 
                    cv.drawMarker(self.cv_image, (int(imagePoint[0][0]),int(imagePoint[0][1])), (0,0,255), cv.MARKER_CROSS, 15, 2)
                    print(dis)
                # print([dis,imagePoint])
                

        imagePoints , _ = cv.projectPoints(clus_centre, np.array([[1.40,-1.36,1.05]], dtype=float), np.array([-0.25, -0.49, -0.14], dtype=float), self.cam_matrix,np.array([[0,0,0,0,0]], dtype=float))
        imagePoints = np.squeeze(imagePoints,1)
        
        
        # print(imagePoints)
        # img = self.cv_image
        
        # print(b)
        # print(self.cv_image.shape)
        # print(img.shape)
#         if img is not None : 
#             for i, row in enumerate(img): 
  
#   # get the pixel values by iterating 
#                 for j, pixel in enumerate(img): 
#                     if(i == j or i+j == img.shape[0]): 
#                 # update the pixel value to black 
#                         img[i][j] = [0, 0, 0] 
        
#******************************
        # for i in imagePoints:
        #     # print(i[0])
        #     if 0<i[0]<360 and 0<i[1]<640 and self.cv_image is not None : 
        #         cv.drawMarker(self.cv_image, (int(i[0]),int(i[1])), (0,0,255), cv.MARKER_CROSS, 15, 2)
#**********************************************



                # self.cv_image[int(i[0]),int(i[1])] = (0,0,255)
        cv.imshow("img",self.cv_image)
        key = cv.waitKey(1)

    # Press 'c' to capture and save the image as a PNG file
        
        if key == ord('c'):
            
            image_filename = f'captured_image_{self.image_count}.png'
            cv.imwrite(image_filename, self.cv_image)
            self.image_count += 1
        # k = cv.waitKey(0)
        # if k == ord('s'): # wait for 's' key to save and exit
        #     cv.imwrite(f'whirldata{im}.png',img)
        #     im+=1
    

        # print(imagePoints)

        # pixel_cords = []
        # for i in clus_centre:
        #     n = np.array([[i[0]],[i[1]],[i[2]],[1]])
        #     p = self.cam_matrix@self.bltocam@np.linalg.inv(self.bltoli)@n
        #     pix = np.array([p[0]/p[2],p[1]/p[2]])
        #     pixel_cords.append(pix)
            # print(pix)
        # for j in pixel_cords:
        #     if 0<=j[0]<=640 and 0<=j[1]<=360:
        #         pass
        #         # print(j)
        # print(clus_centre)
       



def main(args=None):
    rclpy.init(args=args)

    fus = Fuse()


    rclpy.spin(fus)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()