#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


'''
Base class for kalman filter

State vector = {x,y,psi,u,v,r,ax,ay}    # array of size (6x1)

x, y are positions in GCS frame
psi is orientation of BCS wrt GCS frame
r is the yaw rate of BCS wrt GCS.
u, v are surge and sway velocities (required in body frame) : 
Direct integration of accelerations should give those in body frame
ax, ay are body frame accelerations
BUT u,v,ax,ay INCLUDED IN THE STATE ARE IN GCS FRAME. THEY MUST BE IN GCS BECAUSE 
WHILE USING KINEMATICS TO PREDICT THE NEXT STATE WE NEED ALL OF THEM IN SAME FRAME
'''


class KinematicsKF(Node):
    def __init__(self, node_name, frequency=10):
        super().__init__(node_name)
        self.filter_frequency = frequency
        self.num_s = 8        # number of states
        self.num_m = 6        # number of measured states

        ''' Array to store the predicted states '''
        ''' Filtered states include : (x,y,psi,u,v,r,ax,ay) '''
        self.filtered_state = np.zeros(self.num_s)

        ''' Array to store the covariance of filtered states '''
        ''' Filtered states include : (x,y,psi,u,v,r,ax,ay) '''
        self.state_covariance = np.zeros((self.num_s,self.num_s))

        ''' Array to store the measured states '''
        ''' Measured states include : (x,y,psi,r,ax,ay) '''
        self.measured_state = np.zeros(self.num_m)

        ''' Array to store the covariance in measured states [R] '''
        ''' Measured states include : (x,y,psi,r,ax,ay) '''
        self.measurement_noise = np.zeros((self.num_m,self.num_m))

        ''' Array to store the covariance in the process (the kinematics model) '''
        ''' States include : (x,y,psi,u,v,r,ax,ay) '''
        self.process_noise = np.zeros((self.num_s,self.num_s))

        ''' 
        States to publish
        self.x = 0                       # (x,y) in GCS wrt a specified datum
        self.y = 0
        self.psi = 0                     # yaw angle of BCS wrt GCS
        self.r = 0
        self.ax = 0                      # linear acceleration in BCS
        self.ay = 0
        self.u = 0                       # To be published in BCS
        self.v = 0
        '''

        ''' State transition matrix '''
        self.phi_mat = np.identity(self.num_s)
        self.phi_mat[0,3] = self.phi_mat[1,4] = self.phi_mat[2,5] = self.phi_mat[3,6] = self.phi_mat[4,7] = (1/self.filter_frequency)
        self.phi_mat[0,6] = self.phi_mat[1,7] = 0.5*(1/self.filter_frequency)**2

        ''' Setting up the publisher '''
        self.filtered_state_pub = rclpy.create_publisher(Odometry, '/kinematics_kf/state', queue_size=5)
        # Set the publishing frequency to 10Hz
        self.timer = self.create_timer(0.1, self.filter)
        # Initialize the transform broadcaster
        self.filter_broadcaster = TransformBroadcaster(self)

        ''' Message to be published to /kinematics_kf/state topic '''
        self.kf_state = Odometry()

        self.imu_subscribers = []
        self.gnss_subscribers = []
        

    def register_sensors(self, imu_measurements, gnss_measurements):
        self.imu_measurements = imu_measurements
        self.gnss_measurements = gnss_measurements

        for i in range(len(self.imu_measurements)):
            self.imu_subscribers.append(rclpy.create_subscription(Imu, self.imu_measurements[i].topic_name, self.imu_measurements[i].callback, 10))

        for i in range(len(self.gnss_measurements)):
            self.gnss_subscribers.append(rclpy.create_subscription(NavSatFix, self.gnss_measurements[i].topic_name, self.gnss_measurements[i].callback, 10))


    def update(self):
        self.X_minus = self.phi_mat@self.filtered_state                                               # a_priori_state
        self.P_minus = \
            ((self.phi_mat@self.state_covariance)@self.phi_mat.transpose()) + self.process_noise      # a_priori_covariance
    
        # The predicted measurment needs to be transformed using heading at that instant only
        self.yaw = self.X_minus[2]


    def correct_imu(self, imu_N):
        ''' IMU data is coming. So measured states are (psi,r,ax,ay) '''
        imu_N.transform_to_NED()

        self.measured_state[2] = imu_N.orientation_e[2]
        self.measured_state[3] = imu_N.angular_velocity[2] 
        self.measured_state[4] = imu_N.lin_acceleration[0]
        self.measured_state[5] = imu_N.lin_acceleration[1]
        
        C_mat = np.zeros((4,self.num_s))
        C_mat[0,2] = C_mat[1,5] = 1
        C_mat[2:4,6:8] = np.array([[np.cos(self.yaw),np.sin(self.yaw)],[-np.sin(self.yaw),np.cos(self.yaw)]])

        measurement_noise = np.zeros((4,4))
        measurement_noise[0,0] = self.measurement_noise[2,2]    # psi
        measurement_noise[1,1] = self.measurement_noise[3,3]    # r
        measurement_noise[2,2] = self.measurement_noise[4,4]    # ax
        measurement_noise[3,3] = self.measurement_noise[5,5]    # ay

        ''' kalman gain'''
        kg1 = np.dot(self.P_minus,C_mat.transpose())
        kg2 = np.linalg.inv(((C_mat@self.P_minus)@C_mat.transpose()) + measurement_noise)
        kalman_gain = np.dot(kg1,kg2)

        ''' State Corrector '''
        measured_state = np.zeros(4)
        measured_state[0] = self.measured_state[2]
        measured_state[1] = self.measured_state[3]
        measured_state[2] = self.measured_state[4]
        measured_state[3] = self.measured_state[5]

        measurement_error = measured_state - (C_mat@self.X_minus)
        self.filtered_state = self.X_minus + (kalman_gain@measurement_error)

        ''' Covariance Corrector '''
        cc1 = np.identity(self.num_s) - (kalman_gain@C_mat)
        cc2 = (kalman_gain@measurement_noise)@kalman_gain.transpose()
        self.state_covariance = ((cc1@self.P_minus)@cc1.transpose()) + cc2

        self.X_minus = self.filtered_state
        self.P_minus = self.state_covariance


    def correct_gnss(self, gnss_N):
        ''' GPS data is coming. So measured states are (x,y) '''
        self.measured_state[0] = gnss_N.x
        self.measured_state[1] = gnss_N.y
        C_mat = np.zeros((2,self.num_s))
        C_mat[0,0] = C_mat[1,1] = 1

        measurement_noise = np.zeros((2,2))
        measurement_noise[0,0] = self.measurement_noise[0,0]    # x
        measurement_noise[1,1] = self.measurement_noise[1,1]    # y

        ''' kalman gain '''
        kg1 = np.dot(self.P_minus,C_mat.transpose())
        kg2 = np.linalg.inv(((C_mat@self.P_minus)@C_mat.transpose()) + measurement_noise)
        kalman_gain = np.dot(kg1,kg2)

        ''' State Corrector '''
        measured_state = np.zeros(2)
        measured_state[0] = self.measured_state[0]
        measured_state[1] = self.measured_state[1]
        measurement_error = measured_state - (C_mat@self.X_minus)
        self.filtered_state = self.X_minus + (kalman_gain@measurement_error)

        ''' Covariance Corrector '''
        cc1 = np.identity(self.num_s) - (kalman_gain@C_mat)
        cc2 = (kalman_gain@measurement_noise)@kalman_gain.transpose()
        self.state_covariance = ((cc1@self.P_minus)@cc1.transpose()) + cc2

        self.X_minus = self.filtered_state
        self.P_minus = self.state_covariance


    def filter(self):

        # Step-1: Update
        self.update()

        '''states include : (x,y,psi,u,v,r,ax,ay)'''

        for imu_N in self.imu_measurements:
            if imu_N.flag == 1:
                self.correct_imu(imu_N)
                imu_N.flag = 0

        for gnss_N in self.gnss_measurements:
            if gnss_N.flag == 1:
                self.correct_gnss(gnss_N)
                gnss_N.flag = 0
            
        ''' Publish the filtered states with the desired convention '''
        odom_quat = quaternion_from_euler(0, 0, self.filtered_state[2])
        self.kf_state.header.stamp = self.get_clock().now().to_msg()
        self.kf_state.child_frame_id = "filter_base_link"
        point = Point()
        point.x = self.x
        point.y = self.y
        self.kf_state.pose.pose.position = point
        self.kf_state.pose.pose.orientation = odom_quat

        # set the velocity
        psi = self.filtered_state[2]
        rot_mat = np.array([[np.cos(psi),np.sin(psi)],[-np.sin(psi),np.cos(psi)]])
        v_GCS = np.array([self.filtered_state[3], self.filtered_state[4]])
        v_BCS = rot_mat@v_GCS.transpose()
        twist = Twist()
        twist.linear.x = v_BCS[0]
        twist.linear.y = v_BCS[1]
        twist.angular.z = self.angular_velocity[2]

        ''' Publish the dynamic transformation '''
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id="world"
        tfs.child_frame_id = "filter_base_link"
        tfs.transform.translation.x = self.filtered_state[0]
        tfs.transform.translation.y = self.filtered_state[1]
        tfs.transform.translation.z = 0.0

        tfs.transform.rotation.x = odom_quat.x
        tfs.transform.rotation.y = odom_quat.y
        tfs.transform.rotation.z = odom_quat.z
        tfs.transform.rotation.w = odom_quat.w
        
        self.filter_broadcaster.sendTransform(tfs)

        self.imu_flag = 0
        self.gnss_flag = 0


if __name__ == '__main__':
    print("This is just the class")



