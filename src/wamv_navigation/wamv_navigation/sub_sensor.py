#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf_transformations import euler_from_quaternion
import pyproj

'''
Script to log IMU heading and GPS data from WAMV sensors and publish the unfiltered pose to a new topic
'''

class SubscibeSensor(Node):
    def __init__(self):
        super().__init__("sub_sensor")
        # Subscribe to IMU and GPS topics
        self.imu_sub_ = self.create_subscription(Imu,
                         "/wamv/sensors/imu/imu/data", self.imu_callback, 10)
        self.gps_sub_ = self.create_subscription(NavSatFix,
                         "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        # Create a publisher to publish the wamv pose as an Odometry msg
        self.pose_pub_ = self.create_publisher(Odometry, "/wamv_pose", 10)
        # Set the publishing frequency to 10Hz
        self.timer = self.create_timer(0.1, self.pub_pose)
        self.get_logger().info("Sensor Subsciber node has been started")

        self.orientation_q = []
        self.orientation_e = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.x = 0
        self.y = 0
        # For now, fixed value. Need to add it to params server equivalent in ROS2.
        self.origin_gps = [-33.722718, 150.674031]
        # start with datum_flag = 0 if Datum is not decided beforehand, else start with datum_flag = 0
        self.datum_flag = 1

    def imu_callback(self, imu: Imu):
        self.angular_velocity[2] = imu.angular_velocity.z
        self.orientation_q = imu.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        self.orientation_e[0] = euler_from_quaternion(orientation_list)[0]
        self.orientation_e[1] = euler_from_quaternion(orientation_list)[1]
        self.orientation_e[2] = euler_from_quaternion(orientation_list)[2]
        # self.get_logger().info("psi = " + str(self.orientation_e[2]*(180/np.pi)))

    def gps_callback(self, gps: NavSatFix):
        lat = gps.latitude
        lon = gps.longitude

        if self.datum_flag == 0:
            # if the datum_flag = 0, set the datum to current GPS coordinates
            self.origin_gps = [lat, lon]
            self.datum_flag = 1

        self.gps_to_xy_ENU(self.origin_gps[0], self.origin_gps[1], lat, lon)
        # self.get_logger().info("x = " + str(self.x) + " , y = " + str(self.y))

    def gps_to_xy_ENU(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) 
        # by finding side lengths of a right-angle triangle
        # Convert azimuth to radians
        azimuth = np.radians(azimuth)
        # To get (x,y) in ENU frame
        self.y = adjacent = np.cos(azimuth) * distance
        self.x = opposite = np.sin(azimuth) * distance

    def pub_pose(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "wamv_pose_link"
        point = Point()
        point.x = self.x
        point.y = self.y
        msg.pose.pose.position = point
        msg.pose.pose.orientation = self.orientation_q
        twist = Twist()
        twist.angular.z = self.angular_velocity[2]
        self.pose_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SubscibeSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()