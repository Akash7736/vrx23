import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import time
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion




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
        
   
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel =   'wamv/wamv/front_left_camera_link_optical'  #'wamv/wamv/front_left_camera_link'
        to_frame_rel =  'wamv/wamv/lidar_wamv_link'  #

    
        try:
            t1 = self.tf_buffer.lookup_transform(from_frame_rel,to_frame_rel, rclpy.time.Time())
            t2 = self.tf_buffer.lookup_transform('wamv/wamv/front_left_camera_link_optical','wamv/wamv/base_link', rclpy.time.Time())
            quat = [t1.transform.rotation.x,t1.transform.rotation.y,t1.transform.rotation.z,t1.transform.rotation.w]
            print(f"trns : {t1.transform.translation}, rot : {euler_from_quaternion(quat)}")
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