import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import time
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped

lidar_link = 'wamv/wamv/lidar_wamv_link'
camera_link =  'wamv/wamv/front_left_camera_link_optical'
base_link = 'wamv/wamv/base_link'
imu_link = 'wamv/wamv/imu_wamv_link'


class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')
    #     self.declare_parameter('/wamv/wamv/lidar_wamv_link', 'base_link')
    #     self.target_frame = self.get_parameter(
    #   '/wamv/wamv/lidar_wamv_link').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.on_timer)
        # try:
        #     t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
        #     print(t.transform.translation.x)
        # except TransformException as ex:
        #     self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return
        
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