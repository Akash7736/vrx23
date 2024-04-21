#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter
from cv_bridge import CvBridge
import numpy as np

class Vision(Node):
    def __init__(self, model_path):
        super().__init__('Vision')
        self.bridge = CvBridge()
        self.get_logger().info('Running yolo')
        self.model = YOLO(model_path)
        self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.img_callback, 10)
        self.result_pub = self.create_publisher(ParamVec,'/wamv/yolo/results', 10)
        self.img_msg = Image()
        self.trigger = False
        self.loop = self.create_timer(
            timer_period_sec=0.1, callback=self.compute)
    def img_callback(self, img_msg: Image):
        self.img_msg = img_msg
        self.trigger = True
    def compute(self):
        if not self.trigger:
            return
        self.trigger = False
        img = self.bridge.imgmsg_to_cv2(self.img_msg, 'passthrough')
        results = self.model(img)
        pub_msg = ParamVec()
        pub_msg.header = self.img_msg.header
        for infer in results:
            names = infer.names
            for xyxy, cls, conf in zip(infer.boxes.xyxy, infer.boxes.cls, infer.boxes.conf):
                conf = round(float(conf.cpu().numpy()), 3)
                pixs = np.uint32(xyxy.cpu().numpy()).tolist()
                l = int(cls.cpu().numpy())
                param = Parameter()
                param.name = names[l]
                param.value.double_value = conf
                param.value.integer_value = l
                param.value.integer_array_value = pixs
                pub_msg.params.append(param)
        self.result_pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    print('Node Init...')
    vision = Vision(model_path='model/yolov8n_old.pt')
    rclpy.spin(vision)
    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

