#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from vrx_sub.utils import gps_to_xy_ENU, euler_from_quaternion, SyncSubscription
from tf2_ros.transform_broadcaster import TransformBroadcaster
from vrx_sub.config import DATUM, GPS_WAMV_LINK_TO_BASE_LINK_TF


class DummyKf(Node):
    def __init__(self, datum) -> None:
        super().__init__('kalman_filter')
        self.datum = datum
        # self.gps_sub = self.create_subscription(
        #     NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        # self.imu_sub = self.create_subscription(
        #     Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        sensor_df = {'/wamv/sensors/gps/gps/fix': NavSatFix,
                     '/wamv/sensors/imu/imu/data': Imu}
        self.sensor_sub = SyncSubscription(
            self, sensor_df, self.sensor_callback)

        self.kf_pub = self.create_publisher(Odometry, '/wamv/kf/state', 10)
        self.prev_gps, self.curr_gps = None, None

        self.timer = self.create_timer(
            timer_period_sec=0.05, callback=self.compute)

        self.tf_br = TransformBroadcaster(self)
        self.triggered = False

        self.prev_time = self.get_clock().now().nanoseconds

    def compute(self):
        if self.prev_gps is None or self.curr_gps is None:
            return

        if not self.triggered:
            return
        self.triggered = False
        # Calculate Pose, Velocity, Acc

        _, _, yaw = euler_from_quaternion(self.imu.orientation)
        rot_mat = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ])
        trans_to_base = rot_mat @ GPS_WAMV_LINK_TO_BASE_LINK_TF[0]
        pose = gps_to_xy_ENU(datum=self.datum, gps_msg=self.curr_gps) + \
            trans_to_base
        prev_pose = gps_to_xy_ENU(
            datum=self.datum, gps_msg=self.prev_gps) + trans_to_base
        dt, self.prev_time = (self.get_clock().now().nanoseconds - self.prev_time) * 1e-9, self.get_clock().now().nanoseconds
        global_vel = (pose - prev_pose) / dt
        local_vel = rot_mat @ global_vel
        odom = Odometry()

        tf = TransformStamped()

        odom.header.frame_id = 'wamv/wamv/odom'
        odom.header.stamp = self.curr_gps.header.stamp
        odom.child_frame_id = 'wamv/wamv/base_link'
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        odom.pose.pose.position.z = pose[2]
        odom.pose.pose.orientation = self.imu.orientation
        odom.twist.twist.linear.x = local_vel[0]
        odom.twist.twist.linear.y = local_vel[1]
        odom.twist.twist.linear.z = local_vel[2]
        odom.twist.twist.angular = self.imu.angular_velocity

        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.rotation = odom.pose.pose.orientation
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pose
        self.tf_br.sendTransform(tf)
        self.kf_pub.publish(odom)

    def sensor_callback(self, gps_msg: NavSatFix, imu_msg: Imu):
        self.prev_gps, self.curr_gps = self.curr_gps, gps_msg
        self.imu = imu_msg

        self.triggered = True

    # def gps_callback(self, msg: NavSatFix):

    #     self.prev_gps, self.curr_gps = self.curr_gps, msg

    # def imu_callback(self, msg: Imu):
    #     self.imu = msg


def main(args=None):
    rclpy.init(args=args)
    kf = DummyKf(datum=DATUM)
    kf.get_logger().info("Robot Loc Online!!!!")
    rclpy.spin(kf)
    kf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
