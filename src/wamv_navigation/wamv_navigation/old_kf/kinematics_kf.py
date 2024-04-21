#!/usr/bin/env python3
from matplotlib.pyplot import plot
import rclpy
from math import pi, radians, cos, sin
import numpy as np
import pyproj
import tf
from std_msgs.msg import  Float64MultiArray
from std_msgs.msg import  String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler



'''
Base class for kalman filter

State vector = {x,y,psi,u,v,r,ax,ay}    # array of size (6x1)

x, y are positions in ENU frame
psi measured from SBG is with respect to NWU frame, need to get heading wrt ENU
add pi/2 rad to the psi value from sbg to get the value in ENU frame.

r obtained from SBG is with Z axis UP. This r is the yaw rate of BCS wrt GCS.

u, v are surge and sway velocities (required in body frame) : 
Direct integration of accelerations should give those in body frame
ax, ay are body frame accelerations obtained from SBG
BUT u,v,ax,ay INCLUDED IN THE STATE ARE IN GCS FRAME. THEY MUST BE IN GCS
BECAUSE WHILE USING KINEMATICS TO PREDICT THE NEXT STATE WE NEED ALL OF THEM IN SAME FRAME
'''

class kalman_filter():

    def __init__(self, node_name, frequency=10):
        self.node_name = node_name
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
        self.x = 0                       # (x,y) in ENU wrt a specified datum
        self.y = 0
        self.psi = 0                     # yaw angle wrt ENU
        self.r = 0
        self.ax = 0                      # linear acceleration in BCS
        self.ay = 0
        self.u = 0                       # To be published in BCS
        self.v = 0
        '''

        ### Innovation
        self.imuInn = Quaternion()
        self.gnssInn = Quaternion()

        ''' State transition matrix '''
        self.phi_mat = np.identity(self.num_s)
        self.phi_mat[0,3] = self.phi_mat[1,4] = self.phi_mat[2,5] = self.phi_mat[3,6] = self.phi_mat[4,7] = (1/self.filter_frequency)
        self.phi_mat[0,6] = self.phi_mat[1,7] = 0.5*(1/self.filter_frequency)**2

        ''' Setting up the publisher '''
        self.filtered_state_pub = rclpy.Publisher('/kf/state', Odometry, queue_size=5)
        self.IMU_innovation_publisher = rclpy.Publisher('/kf/imu_innovation', Quaternion, queue_size=10)
        self.GNSS_innovation_publisher = rclpy.Publisher('/kf/gnss_innovation', Quaternion, queue_size=10)
        self.filter_broadcaster = tf.TransformBroadcaster()


        ''' Message to be published to /kf/state topic '''
        self.kf_state = Odometry()

        self.imu_subscribers = []
        self.gnss_subscribers = []
        

    def register_sensors(self, imu_measurements, gnss_measurements):
        self.imu_measurements = imu_measurements
        self.gnss_measurements = gnss_measurements

        for i in range(len(self.imu_measurements)):
            self.imu_subscribers.append(rclpy.Subscription(self.imu_measurements[i].topic_name, Imu, self.imu_measurements[i].callback))

        for i in range(len(self.gnss_measurements)):
            self.gnss_subscribers.append(rclpy.Subscription(self.gnss_measurements[i].topic_name, NavSatFix, self.gnss_measurements[i].callback))


    def update(self):
        self.X_minus = np.dot(self.phi_mat,self.filtered_state)                  # a_priori_state
        # rospy.loginfo(f"u : {X_minus[3]}, v : {X_minus[4]}")
        # rospy.loginfo(f"sbg_ax : {self.measured_state[4]}, sbg_ay : {self.measured_state[5]}")
        self.P_minus = \
            np.dot(np.dot(self.phi_mat,self.state_covariance),self.phi_mat.transpose()) + self.process_noise      # a_priori_covariance
    
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
        C_mat[2:4,6:8] = np.array([[cos(self.yaw),sin(self.yaw)],[-sin(self.yaw),cos(self.yaw)]])

        measurement_noise = np.zeros((4,4))
        measurement_noise[0,0] = self.measurement_noise[2,2]    # psi
        measurement_noise[1,1] = self.measurement_noise[3,3]    # r
        measurement_noise[2,2] = self.measurement_noise[4,4]    # ax
        measurement_noise[3,3] = self.measurement_noise[5,5]    # ay

        ''' kalman gain'''
        kg1 = np.dot(self.P_minus,C_mat.transpose())
        kg2 = np.linalg.inv(np.dot(np.dot(C_mat,self.P_minus),C_mat.transpose()) + measurement_noise)
        kalman_gain = np.dot(kg1,kg2)    

        ''' State Corrector '''
        measured_state = np.zeros(4)
        measured_state[0] = self.measured_state[2]
        measured_state[1] = self.measured_state[3]
        measured_state[2] = self.measured_state[4]
        measured_state[3] = self.measured_state[5]

        # rospy.loginfo(f"sbg_ax : {measured_state[2]}, sbg_ay : {measured_state[3]}")
        measurement_error = measured_state - np.dot(C_mat,self.X_minus)
        self.imuInn.x = measurement_error[0]
        self.imuInn.y = measurement_error[1]
        self.imuInn.z = measurement_error[2]
        self.imuInn.w = measurement_error[3]
        self.filtered_state = self.X_minus + np.dot(kalman_gain,measurement_error)
        # rospy.loginfo(f"r from filter : {self.filtered_state[5]}")

        ''' Covariance Corrector '''
        cc1 = np.identity(self.num_s) - np.dot(kalman_gain,C_mat)
        cc2 = np.dot(np.dot(kalman_gain,measurement_noise),kalman_gain.transpose())
        self.state_covariance = np.dot(np.dot(cc1,self.P_minus),cc1.transpose()) + cc2

        self.X_minus = self.filtered_state
        self.P_minus = self.state_covariance


    def correct_gnss(self, gnss_N):
        ''' GPS data is coming. So measured states are (x,y) '''
        self.measured_state[0] = gnss_N.x
        self.measured_state[1] = gnss_N.y
        # rospy.loginfo("gpsxy %d %d ", self.measured_state[0], self.measured_state[1])
        C_mat = np.zeros((2,self.num_s))
        C_mat[0,0] = C_mat[1,1] = 1

        measurement_noise = np.zeros((2,2))
        measurement_noise[0,0] = self.measurement_noise[0,0]    # x
        measurement_noise[1,1] = self.measurement_noise[1,1]    # y

        ''' kalman gain '''
        kg1 = np.dot(self.P_minus,C_mat.transpose())
        kg2 = np.linalg.inv(np.dot(np.dot(C_mat,self.P_minus),C_mat.transpose()) + measurement_noise)
        kalman_gain = np.dot(kg1,kg2)

        ''' State Corrector '''
        measured_state = np.zeros(2)
        measured_state[0] = self.measured_state[0]
        measured_state[1] = self.measured_state[1]
        measurement_error = measured_state - np.dot(C_mat,self.X_minus)
        self.gnssInn.x = measurement_error[0]
        self.gnssInn.y = measurement_error[1]
        self.filtered_state = self.X_minus + np.dot(kalman_gain,measurement_error)
        # rospy.loginfo(f"r from X minus : {X_minus[5]}")

        # rospy.loginfo(f"r from filter again : {self.filtered_state[5]}")

        ''' Covariance Corrector '''
        cc1 = np.identity(self.num_s) - np.dot(kalman_gain,C_mat)
        cc2 = np.dot(np.dot(kalman_gain,measurement_noise),kalman_gain.transpose())
        self.state_covariance = np.dot(np.dot(cc1,self.P_minus),cc1.transpose()) + cc2

        self.X_minus = self.filtered_state
        self.P_minus = self.state_covariance


    def filter(self):

        rclpy.init_node(self.node_name, disable_signals=True)      

        rclpy.sleep(5)

        while not rospy.is_shutdown():

            # Step-1: Update
            self.update()

            '''states include : (x,y,psi,u,v,r,ax,ay)'''

            for imu_N in self.imu_measurements:
                if imu_N.flag == 1:
                    self.correct_imu(imu_N)
                    imu_N.flag = 0

            # rospy.loginfo(f"u : {X_minus[3]}, v : {X_minus[4]}")
            for gnss_N in self.gnss_measurements:
                if gnss_N.flag == 1:
                    self.correct_gnss(gnss_N)
                    gnss_N.flag = 0
            
            
            ''' Publish the filtered states with the desired convention '''
            odom_quat = quaternion_from_euler(0, 0, self.filtered_state[2])
            current_time = rospy.Time.now()
    
            self.filter_broadcaster.sendTransform(
            (self.filtered_state[0], self.filtered_state[1], 0.),
            odom_quat,
            current_time,
            "base_link",
            "map")
    
            self.kf_state.header.stamp = current_time
            # self.kf_state.header.frame_id = "map"
            self.kf_state.child_frame_id = "base_link"
    
            # set the position
            self.kf_state.pose.pose = Pose(Point(self.filtered_state[0], self.filtered_state[1], 0.), Quaternion(*odom_quat))
        
            # set the velocity
            psi = self.filtered_state[2]
            rot_mat = np.array([[cos(psi),sin(psi)],[-sin(psi),cos(psi)]])
            v_GCS = np.array([self.filtered_state[3], self.filtered_state[4]])
            v_BCS = np.dot(rot_mat,v_GCS.transpose())

            self.kf_state.twist.twist = Twist(Vector3(v_BCS[0], v_BCS[1], 0), Vector3(0, 0, self.filtered_state[5]))
        
            # publish the message
            self.filtered_state_pub.publish(self.kf_state)
            self.IMU_innovation_publisher.publish(self.imuInn)
            self.GNSS_innovation_publisher.publish(self.gnssInn) 

            self.imu_flag = 0
            self.gnss_flag = 0

            rospy.Rate(self.filter_frequency).sleep()
        


if __name__ == '__main__':
    print("This is just the class, please instantiate the onjects of this class along with imu objects using filter_base file.py")