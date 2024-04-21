import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import numpy as np
import time
import pyproj
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu 
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
import sensor_msgs_py.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

lidar_link = 'wamv/wamv/lidar_wamv_link'
camera_link =  'wamv/wamv/front_left_camera_link_optical'
base_link = 'wamv/wamv/base_link'
imu_link = 'wamv/wamv/imu_wamv_link'
worldlink = 'map'
origin = [-33.72277565710174,150.673991252778]

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')
    #     self.declare_parameter('/wamv/wamv/lidar_wamv_link', 'base_link')
    #     self.target_frame = self.get_parameter(
    #   '/wamv/wamv/lidar_wamv_link').get_parameter_value().string_value
        self.xp = 0.0
        self.yp = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.clocktime  = 0
        self.heightthresh = 1
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tfb_ = TransformBroadcaster(self)
        self.filpublisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        self.subscriptionvoxel = self.create_subscription(PointCloud2,'/myvoxel',self.voxel_callback,10)
        self.subscriptionvoxel 
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning
        
        self.timer = self.create_timer(1.0, self.on_timer)
        # try:
        #     t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
        #     print(t.transform.translation.x)
        # except TransformException as ex:
        #     self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return
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
    def quaternion_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
    
    def transform_point(self, transformation, point_wrt_source):
        point_wrt_target = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_source), transformation).point
        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]
    
    def voxel_callback(self, msg:PointCloud2):
        self.clocktime = msg.header.stamp
  
        pt_arr = []
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
            t1 = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time())
            pt2 = self.transform_point(t1, point_wrt_lidar)
            point_wrt_base = Point()
            point_wrt_base.x = float(pt2[0])
            point_wrt_base.y = float(pt2[1])
            point_wrt_base.z = float(pt2[2])

            from_frame_rel = base_link
            to_frame_rel = worldlink
            t2 = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time()) 
            point_wrt_world = self.transform_point(t2, point_wrt_base)           # print([pt_x,pt_y,pt_z])
            if point_wrt_world[2] > self.heightthresh:
                pt_arr.append([point_wrt_lidar.x,point_wrt_lidar.y,point_wrt_lidar.z])
        pt_arr = np.array(pt_arr)
        header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        header.stamp = self.clocktime
        header.frame_id = base_link
        pc_msg = point_cloud2.create_cloud_xyz32(header, pt_arr)
        
        # Publish the PointCloud2 message
        self.filpublisher.publish(pc_msg)
        print(pt_arr.shape)
        print("Points Published ...........")

    def gps_callback(self,data:NavSatFix):
       
        geo_lat = float(data.latitude)  
        geo_lon = float(data.longitude)

        self.xp,self.yp = self.gps2enu(origin[0],origin[1],geo_lat,geo_lon)
        
        cord = [self.xp,self.yp]

    def imu_callback(self,data:Imu):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw = euler[2] # yaw
        self.roll = euler[0]
        self.pitch = euler[1]
        # self.R = self.euler_to_rotation_matrix(self.roll, self.pitch, self.yaw)
        # baselink_coordinates = np.array([[self.xp], [self.yp], [0]])
        # self.world_coordinates = np.dot(self.R, baselink_coordinates)
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id="map"
        tfs._child_frame_id = base_link
        tfs.transform.translation.x = self.xp
        tfs.transform.translation.y = self.yp
        tfs.transform.translation.z = 0.0  

        r = R.from_euler('xyz',[self.roll,self.pitch,self.yaw])

        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]

        self.tfb_.sendTransform(tfs)    
        # print("Transform Broadcasted ...")
        # t = self.tf_buffer.lookup_transform(worldlink,base_link, rclpy.time.Time())

        # print(t.transform.translation)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = base_link
        to_frame_rel = imu_link  #wamv/wamv/front_left_camera_link_optical'

    
        try:
            point_wrt_source = Point()
            point_wrt_source.x = 0.1
            point_wrt_source.y = 1.2
            point_wrt_source.z = 2.3
            # t1 = self.tf_buffer.lookup_transform('wamv/wamv/lidar_wamv_link','wamv/wamv/base_link', rclpy.time.Time())
            t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel, rclpy.time.Time())
            point_wrt_target = self.transform_point(t, point_wrt_source)
            print(f"point w.r.t source : {point_wrt_source}")
            print(f"point w.r.t target : {point_wrt_target}")
            
            rotmat = self.quaternion_rotation_matrix([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
            rvec, _ = cv2.Rodrigues(rotmat)
            rvec = np.array(rvec).reshape((1,3))[0]
            tvec = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
            # print(f"tvec :{tvec} rvec : {rvec} rotmat : {rotmat}")
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


def main(args=None):
    rclpy.init(args=args)

    fl = FrameListener()


    rclpy.spin(fl)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()