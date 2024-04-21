#!/usr/bin/env python3
from tf2_ros.buffer import Buffer
import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from utils import quaternion_to_rotation_matrix
import numpy as np
buffer = Buffer()

class Tf_Reader(Node):
    def __init__(self):
        super().__init__('Tf_Reader')

        self.buffer = Buffer()
        self.l = TransformListener(self.buffer, self)

        self.t = self.create_timer(timer_period_sec=5, callback=self.output)

        self.frame_df ={
            'wamv/wamv/lidar_wamv_link': 'wamv/wamv/base_link',
            'wamv/wamv/gps_wamv_link' : 'wamv/wamv/base_link',
            'wamv/wamv/far_left_camera_link': 'wamv/wamv/base_link',
            'wamv/wamv/front_left_camera_link': 'wamv/wamv/base_link',
            'wamv/wamv/front_right_camera_link': 'wamv/wamv/base_link',
            'wamv/wamv/base_link' : 'wamv/wamv/front_left_camera_link',
            # 'wamv/wamv/lidar_wamv_link' : 'wamv/wamv/front_left_camera_link',
        }
    def get_tf(self, from_frame_rel, to_frame_rel):
        try:
            t = self.buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            trans_mat = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
            rot_mat = quaternion_to_rotation_matrix(t.transform.rotation)
            return trans_mat, rot_mat
        except Exception as e:
            print(e)
            return None
    def output(self):
        for to_tf in self.frame_df:
            tf = self.get_tf(to_frame_rel=to_tf, from_frame_rel=self.frame_df[to_tf])
            if tf is None:
                print('Error', to_tf, self.frame_df[to_tf])
                continue
            print('\n')
            # print(to_tf, self.frame_df[to_tf])

            print(f'{to_tf[10:]}_to_{self.frame_df[to_tf][10:]}_TF'.upper(),f' = (np.array({tf[0].tolist()}), np.array({tf[1].tolist()}))')
            # print(tf[0])
            # print(tf[1])
            print('\n')
            

def main(args=None):
    rclpy.init(args=args)
    tf_Reader = Tf_Reader()
    rclpy.spin(tf_Reader)
    tf_Reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
