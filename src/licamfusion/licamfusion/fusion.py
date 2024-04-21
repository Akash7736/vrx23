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
import sensor_msgs_py.point_cloud2 as point_cloud2
# from sensor_msgs import point_cloud2
import cv2 as cv 
import pyproj
from cv_bridge import CvBridge
bridge = CvBridge()
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu 
from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy as rnp
# import keyboard
# import pynput
# from pynput.keyboard import Key, Listener
origin = [-33.72277565710174,150.673991252778]
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
        self.xp = 0
        self.yp = 0 
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
        self.filpublisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.subscriptiontf = self.create_subscription(TFMessage,'/tf',self.tf_callback,10)
        self.subscriptiontf  # prevent unused variable warning
        self.subscriptionimg = self.create_subscription(Image,'/wamv/sensors/cameras/far_left_camera_sensor/image_raw',self.image_callback,10)
        self.subscriptionimg  # prevent unused variable warning
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning

        # self.subscriptionvoxel = self.create_subscription(PointCloud2,'/wamv/sensors/lidars/lidar_wamv_sensor/points',self.voxel_callback,10)
        self.subscriptionvoxel = self.create_subscription(PointCloud2,'/myvoxel',self.voxel_callback,10)
        self.subscriptionvoxel 
        self.cv_image = None
        self.image_count = 0
        self.K = np.array([762.7223205566406, 0.0, 640.0, 0.0,
                          762.7223110198975, 360.0, 0.0, 0.0, 1.0]).reshape((3, 3))

        self.baselink_to_lidar_rot = np.array([
            [0.9902680687415704, -0.0, -0.13917310096006544],
            [0.0, 1.0, -0.0],
            [0.13917310096006544, 0.0, 0.9902680687415704]])
        self.baselink_to_lidar_trans = np.array(
            [-0.4426760663909814, 0.0, -1.8799036944068728])

        self.baselink_to_cam_rot = np.array([
            [0.9659258262890682, 0.0, 0.2588190451025209],
            [0.0, 1.0, 0.0],
            [-0.2588190451025209, 0.0, 0.9659258262890682]])
        self.baselink_to_cam_trans = np.array([0.75, 0.1, 1.5])

    def world_to_cam(self, points, rot, trans):
        world_to_baselink  = (rot.T @ (points - trans).T).T
        return (self.baselink_to_cam_rot.T @ (world_to_baselink - self.baselink_to_cam_trans).T).T 
    
    def lidar_to_baselink(self, points):
        return (self.baselink_to_lidar_rot.T @
                             (points - self.baselink_to_lidar_trans).T).T
    
    def baselink_to_world(self, points, rot, trans):
        return (rot @ points.T).T + trans
    


    def transform_to_base_link(self,lidar_points, translation, rotation_matrix):
        """
        Transform Lidar points from Lidar frame to base_link frame.

        Parameters:
        - lidar_points: Nx3 array representing the Lidar points (x, y, z) in Lidar frame.
        - translation: 1x3 array representing the translation vector.
        - rotation_matrix: 3x3 array representing the rotation matrix.

        Returns:
        - base_link_points: Nx3 array representing the transformed points in base_link frame.
        """
        # Add a column of ones to lidar_points to represent homogeneous coordinates
        lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

        # Apply the rotation matrix
        rotated_points = np.dot(rotation_matrix, lidar_points_homogeneous.T).T

        # Apply the translation
        base_link_points = rotated_points + translation

        return base_link_points
    
    def transform_from_base_link(self,base_link_points, translation, rotation_matrix):
        """
        Transform points from base_link frame to imu_link frame.

        Parameters:
        - base_link_points: Nx3 array representing the points (x, y, z) in base_link frame.
        - translation: 1x3 array representing the translation vector.
        - rotation_matrix: 3x3 array representing the rotation matrix.

        Returns:
        - imu_points: Nx3 array representing the transformed points in imu_link frame.
        """
        # Add a column of ones to base_link_points to represent homogeneous coordinates
        base_link_points_homogeneous = np.hstack((base_link_points, np.ones((base_link_points.shape[0], 1))))

        # Apply the rotation matrix
        rotated_points = np.dot(rotation_matrix, base_link_points_homogeneous.T).T

        # Apply the translation
        imu_points = rotated_points + translation

        return imu_points
    
    def pcd_to_world(self,lidarpoint):
        lidar_to_bl_tr = np.array([0.7, 0.0, 1.8])
        lidar_to_bl_rot = np.array([[-0.99026807 , 0    ,      0.1391731 ],
                                    [ 0     ,   -1  ,     0        ],
                                    [ 0.1391731  , 0 ,     0.99026807]])
        bslpoints = self.transform_to_base_link(lidarpoint,lidar_to_bl_tr,lidar_to_bl_rot)
        bl_to_world_tr =  np.array([[-self.xp], [-self.yp], [0]])
        bl_to_world_rot = self.euler_to_rotation_matrix(-self.roll, -self.pitch, -self.yaw)
        worldpoint = self.transform_from_base_link(bslpoints,bl_to_world_tr,bl_to_world_rot)
        return worldpoint
    
    def apply_mask(self, point_cloud):
        # Assuming point_cloud is an array with shape (N, 3) representing Lidar points
        # transformed_array = np.apply_along_axis(pcd_to_world, axis=1, arr=original_array)
        transformed_array = np.apply_along_axis(self.pcd_to_world, axis=1, arr=point_cloud)

# Filter the original array based on the condition that the third value in the transformed array is greater than 7
        
        # filtered_original_array = point_cloud[transformed_array[:, 2] > 0]
        filtered_original_array = point_cloud

        # world_points = np.apply_along_axis(self.pcd_to_world, axis=1, arr=point_cloud)
        
        # Create a mask where the third value of each row is greater than 1
        # mask = world_points[:, 2] > 1

        # Apply the mask to filter the points
        # filtered_points = point_cloud[mask]

        return filtered_original_array
    

    def apply_mask_and_publish(self, point_cloud):
        # filtered_points = point_cloud[point_cloud[:, 2] > 0.5]

        # transformed_array = np.apply_along_axis(self.pcd_to_world, axis=1, arr=point_cloud)
        # filtered_original_array = point_cloud[transformed_array[:, 2] > 0]

        # Assuming point_cloud is an array with shape (N, 3) representing Lidar points
        world_points = np.apply_along_axis(self.pcd_to_world, axis=1, arr=point_cloud)
        
        # Create a mask where the third value of each row is greater than 1
        mask = world_points[:, 2] > 1

        # Apply the mask to filter the points
        filtered_points = point_cloud[mask]

        # Convert filtered_points to PointCloud2 message
        pc_msg = self.numpy_array_to_pointcloud2(filtered_points)

        # Publish the PointCloud2 message
        self.filpublisher.publish(pc_msg)

    def create_pointcloud2_msg(self, points):
        header = self.get_msg_header()
        fields = [
            PointField('x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField('y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField('z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = point_cloud2.create_cloud_xyz32(header, fields, points)
        return pc_msg
    def numpy_array_to_pointcloud2(self,numpy_array):

        # pc = numpy_array
        # pc_array = np.zeros(len(pc), dtype=[
        #     ('x', np.float32),
        #     ('y', np.float32),
        #     ('z', np.float32),
        #     ('vectors', np.float32, (3,))
        # ])
        # pc_array['x'] = pc[:, 0]
        # pc_array['y'] = pc[:, 1]
        # pc_array['z'] = pc[:, 2]
        # pc_array['vectors'] = np.zeros((len(pc), 3), dtype=np.float32) 


        # pc_msg = rnp.msgify(PointCloud2, pc_array)

        # Create header
        header = self.get_msg_header()
        # header.stamp = rospy.Time.now()
        header.frame_id = "wamv/wamv/base_link"  # Replace with your desired frame ID

        # Reshape the array to a flat list
        # flat_array = numpy_array.flatten()
        # fields = [
        #     PointField('x', offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField('y', offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField('z', offset=8, datatype=PointField.FLOAT32, count=1),
        # ]
        # Create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud_xyz32(header, numpy_array)

        return pc2_msg
    
    def get_msg_header(self):
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.stamp = self.clocktime
        header.frame_id = 'your_frame_id'  # Change this to the appropriate frame ID
        return header


        
    def euler_to_rotation_matrix(self,roll, pitch, yaw):
        """
        Convert roll, pitch, and yaw angles to a 3x3 rotation matrix.
        """
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        # Combine the rotation matrices
        R = np.dot(R_z, np.dot(R_y, R_x))

        return R
    
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
    
    def gps_callback(self,data):
       
        geo_lat = float(data.latitude)  
        geo_lon = float(data.longitude)

        self.xp,self.yp = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
        
        cord = [self.xp,self.yp]

    def imu_callback(self,data):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw = euler[2] # yaw
        self.roll = euler[0]
        self.pitch = euler[1]
        self.R = self.euler_to_rotation_matrix(self.roll, self.pitch, self.yaw)
        baselink_coordinates = np.array([[self.xp], [self.yp], [0]])
        self.world_coordinates = np.dot(self.R, baselink_coordinates)


        
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

    def XYZ_to_UV(self,K, points: np.ndarray):
        trans_points = np.zeros((len(points), 3))
        trans_points[:, 2] = points[:, 0]
        trans_points[:, 0] = points[:, 1] * -1
        trans_points[:, 1] = points[:, 2] * -1

        pixels = np.dot(K, trans_points.T).T
        pixels = np.int32(pixels[:, :2] / pixels[:, 2].reshape(-1, 1))

        return pixels


  

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
        self.cv_image = cv.resize(self.cv_image, (1280,720))
        self.cv_image = cv.cvtColor(self.cv_image,cv.COLOR_BGRA2BGR)
        # (b, g, r) = self.cv_image[0, 0]
        # print(b)
# 
    def resize_lidar_points_from_msg(self,msg):
        """
        Read Lidar points from a PointCloud2 message, resize to size N * 3, and return as a NumPy array.

        Parameters:
        - msg: PointCloud2 message

        Returns:
        - resized_lidar_points: NumPy array of shape (N, 3) containing resized Lidar points (x, y, z)
        """
        lidar_points = []

        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            lidar_points.append(point)

        lidar_points = np.array(lidar_points)

        N = lidar_points.shape[0]
        resized_lidar_points = np.zeros((N, 3))

        if N > 0:
            resized_lidar_points[:, :] = lidar_points[:, :3]

        return resized_lidar_points

    def voxel_callback(self, msg:PointCloud2):
        self.clocktime = msg.header.stamp


        xyz = np.array(list(point_cloud2.read_points(msg,field_names = ("x", "y", "z"), skip_nans=True)))
        
        # print(f"shape of pcd : {xyz2.shape}")
        # self.apply_mask_and_publish(xyz)
        xyz = [list(ele) for i,ele in enumerate(xyz)]
        # print(xyz)
     
  
        pt_arr = []
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            # print([pt_x,pt_y,pt_z])
            pt_arr.append([pt_x,pt_y,pt_z])
        pt_arr = np.array(pt_arr)
        self.apply_mask_and_publish(pt_arr)
        # print(pt_arr)

        # model = DBSCAN(eps=3, min_samples=2)
        # model.fit_predict(xyz)
        db = DBSCAN(eps=0.6, min_samples=8).fit(xyz)
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
        tvec = np.array([0.09999999999999987, -0.2768367956315947, -0.12594200484520973])
        rvec = np.array([-1.26449443 , 1.11873017 , 1.11873017])

        # imagePoints , _ = cv.projectPoints(clus_centre, np.array([[1.40,-1.36,1.05]], dtype=float), np.array([-0.25, -0.49, -0.14], dtype=float), self.cam_matrix,np.array([[0,0,0,0,0]], dtype=float))

        imagePoints , _ = cv.projectPoints(clus_centre, rvec, tvec, self.cam_matrix,np.array([[0,0,0,0,0]], dtype=float))
        # tvec is the translation of Lidar frame from the camera world origin frame
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