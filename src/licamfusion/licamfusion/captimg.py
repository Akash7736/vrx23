#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node  
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from std_msgs.msg import Header
import numpy as np
from quattoeul import tf2tfmat
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2
import cv2 as cv 
from cv_bridge import CvBridge
bridge = CvBridge()

# import keyboard
# import pynput
# from pynput.keyboard import Key, Listener



class Fuse(Node):

    def __init__(self):
        super().__init__('capturer')

        self.subscriptionimg = self.create_subscription(Image,'/wamv/sensors/cameras/far_left_camera_sensor/image_raw',self.image_callback,10)
        self.subscriptionimg  # prevent unused variable warning
 
        self.cv_image = None
        self.image_count = 87

        self.timer = self.create_timer(3, self.on_timer)

    def image_callback(self, msg):
 
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        # print(self.cv_image .shape)
        # self.cv_image = cv.resize(self.cv_image, (640,360))
        self.cv_image = cv.cvtColor(self.cv_image,cv.COLOR_BGRA2BGR)

    def on_timer(self):
        filename = f"poll3/scan_dock_{self.image_count}.png"
        self.image_count += 1
        cv.imwrite(filename, self.cv_image)
        print(self.image_count)
        # (b, g, r) = self.cv_image[0, 0]
        # print(b)
# 
        



def main(args=None):
    rclpy.init(args=args)

    fus = Fuse()


    rclpy.spin(fus)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()