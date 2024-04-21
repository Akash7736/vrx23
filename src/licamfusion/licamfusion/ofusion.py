#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node  
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
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
        self.subscriptionimg = self.create_subscription(Image,'/wamv/sensors/cameras/far_left_camera_sensor/image_raw',self.image_callback,10)
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
        # if tfdict.get('wamv/wamv/base_link to wamv/wamv/base_link/front_left_camera_sensor')is not None: print(tfdict['wamv/wamv/base_link to wamv/wamv/base_link/front_left_camera_sensor'].translation) 
        
        # data._transforms[0].

    def image_callback(self, msg):
 
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        # print(self.cv_image .shape)
        self.cv_image = cv.resize(self.cv_image, (640,360))
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
        db = DBSCAN(eps=0.3, min_samples=8).fit(xyz)
        # pred = model.fit_predict(np.array(pt_arr))
        # clustering = DBSCAN(eps=7, min_samples=2).fit(np.array(pt_arr))
        # print(model.labels_,len(model.labels_),model.labels_.max())
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


        pcd_point_cloud = o3d.data.PCDPointCloud()
        pcd = o3d.io.read_point_cloud(msg)

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        self.cluspub.publish(inlier_cloud)

        # max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([pcd],
        #                                 zoom=0.455,
        #                                 front=[-0.4999, -0.1659, -0.8499],
        #                                 lookat=[2.1813, 2.0619, 2.0999],
        #                                 up=[0.1204, -0.9852, 0.1215])






        imagePoints , _ = cv.projectPoints(clus_centre, np.array([[1.40,-1.36,1.05]], dtype=float), np.array([-0.25, -0.49, -0.14], dtype=float), self.cam_matrix,np.array([[0,0,0,0,0]], dtype=float))
        imagePoints = np.squeeze(imagePoints,1)
        print(imagePoints)
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
        for i in imagePoints:
            # print(i[0])
            if 0<i[0]<360 and 0<i[1]<640 and self.cv_image is not None : 
                cv.drawMarker(self.cv_image, (int(i[0]),int(i[1])), (0,0,255), cv.MARKER_CROSS, 15, 2)

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