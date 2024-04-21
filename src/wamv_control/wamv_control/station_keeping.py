#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import pyproj
from tf_transformations import euler_from_quaternion

class StationKeeping(Node):

    def __init__(self):
        super().__init__("station_keeping")
        # subscribe to /wamv_pose topic
        self.pose_sub_ = self.create_subscription(Odometry,
                         "/wamv_pose", self.pose_callback, 10)
        # subscribe to the /stationkeeping/goal topic
        self.sk_goal_sub_ = self.create_subscription(PoseStamped,
                         "/vrx/stationkeeping/goal", self.sk_goal_callback, 10)
        # publishers for thruster_cmd topics
        self.publisher_lf = self.create_publisher(Float64, '/wamv/thrusters/leftfore/thrust', 10)
        self.publisher_rf = self.create_publisher(Float64, '/wamv/thrusters/rightfore/thrust', 10)
        self.publisher_la = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        self.publisher_ra = self.create_publisher(Float64, '/wamv/thrusters/rightaft/thrust', 10)
        # Set the publishing frequency to 10Hz
        self.timer = self.create_timer(0.1, self.thrust_pub)
        # PD controller gains
        self.Kp = -100*np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.Kd = np.array([[0.1, 0, 0],[0, 0.1, 0],[0, 0, 0.1]])

        # Goal pose : (x, y, psi)
        self.goal_pose = np.array([0., 0, 0])
        # For now, fixed value. Need to add it to params server equivalent in ROS2.
        self.origin_gps = [-33.722718, 150.674031]
        # Wamv_pose : (x, y, psi)
        self.wamv_pose = np.array([0., 0, 0])

        # Thrust command messages
        self.msg_lf = Float64()
        self.msg_rf = Float64()
        self.msg_la = Float64()
        self.msg_ra = Float64()

        # Thrust Allocation (CHECK!!)
        # [lf, rf, la, ra]
        self.TA_matrix = np.array([[0.707, 0.707, 0.707, 0.707],
                                    [-0.707, 0.707, 0.707, -0.707],
                                    [-1.626,  1.626,  -2.333,  2.333]])

    def goal_xy_from_GPS(self, goal_lat, goal_lon):
        # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(self.origin_gps[1], self.origin_gps[0],
                                        goal_lon, goal_lat)
        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) 
        # by finding side lengths of a right-angle triangle
        # Convert azimuth to radians
        azimuth = np.radians(azimuth)
        # To get (x,y) in ENU frame
        y = adjacent = np.cos(azimuth) * distance
        x = opposite = np.sin(azimuth) * distance
        return x, y
    
    def pose_callback(self, msg:Odometry):
        quat = msg.pose.pose.orientation
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        self.wamv_pose[2] = euler_from_quaternion(orientation_list)[2]
        self.wamv_pose[0] = msg.pose.pose.position.x + 0.26*np.cos(self.wamv_pose[2]) 
        self.wamv_pose[1] = msg.pose.pose.position.y + 0.26*np.sin(self.wamv_pose[2])

    def sk_goal_callback(self, msg:PoseStamped):
        goal_lat = msg.pose.position.x
        goal_lon = msg.pose.position.y
        goal_quat = msg.pose.orientation
        self.goal_pose[0], self.goal_pose[1] = self.goal_xy_from_GPS(goal_lat, goal_lon)
        orientation_list = [goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w]
        self.goal_pose[2] = euler_from_quaternion(orientation_list)[2]

    # Function to map the thrust to the commands in range(-1,1)
    def inverse_glf_map(self, T):
        if T >= 250:
            T = 250
        elif T < -100:
            T = -100
        elif 0.06 <= T < 1.2:
            T = 0.06

        if 1.2 <= T <= 250:
            A = 0.01
            K = 59.82
            B = 5.0
            nu = 0.38
            C = 0.56
            M = 0.
        if -100 <= T <= 0.06:
            A = -199.13
            K = -0.09
            B = 8.84
            nu = 5.34
            C = 0.99
            M = -0.57

        cmd = M - (1/B)*np.log((((K-A)/(T-A))**nu)-C)
        return cmd

    def thrust_pub(self):
        pose_error = self.wamv_pose - self.goal_pose 
        self.get_logger().info("error_x = " + str(pose_error[0]) + " , error_y = " + str(pose_error[1]))

        # Rotation matrix ( to convert a vector in global frame to local frame )
        psi = self.wamv_pose[2]
        rot_mat = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])

        global_force = self.Kp @ pose_error

        local_force = rot_mat @ global_force

        thrust = np.linalg.pinv(self.TA_matrix) @ local_force

        self.msg_lf.data = self.inverse_glf_map(thrust[0])*100
        self.msg_rf.data = self.inverse_glf_map(thrust[1])*100
        self.msg_la.data = self.inverse_glf_map(thrust[2])*100
        self.msg_ra.data = self.inverse_glf_map(thrust[3])*100

        self.publisher_lf.publish(self.msg_lf)
        self.publisher_rf.publish(self.msg_rf)
        self.publisher_la.publish(self.msg_la)
        self.publisher_ra.publish(self.msg_ra)


def main(args=None):
    rclpy.init(args=args)
    node = StationKeeping()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        



        
        
    