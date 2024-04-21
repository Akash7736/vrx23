import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np
import math

from pyproj import Geod
from scipy.optimize import curve_fit
# from controller_wamv import PDController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import tf2_ros as tf
import time


# pdc = PDController(0.1,0.2)
goal = [0,0]
origin = [0,0] 
origin_flag = 0
coa = 1 # Circle of Acceptance 
delta = 2 # Look Ahead Distanceyp 
xp = 0
yp = 0
yaw = 0

last_goal = [0,0]
thrust_val = [1500,1500,1500,1500]


g = Geod(ellps = 'WGS84')

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_lf = self.create_publisher(Float64, '/wamv/thrusters/leftfore/thrust', 10)
        self.publisher_rf = self.create_publisher(Float64, '/wamv/thrusters/rightfore/thrust', 10)
        self.publisher_la = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        self.publisher_ra = self.create_publisher(Float64, '/wamv/thrusters/rightaft/thrust', 10)
        self.geopub = self.create_publisher(Float64, '/mytopic/imu/yaw', 10)
        self.yawpubrad = self.create_publisher(Float64, '/mytopic/geo/cord', 10)

        self.kp = 0.1
        self.kd = 0.01


        self.TA_matrix = np.array([ [0, 0, 0],   # pwm_mat = TA_mat * k_mat * error_mat
                                    [0, 0, 0],
                                    [0, 0, 0],  # 4 * 3
                                    [0, 0, 0]])
        
        self.pseudo_TA_inv = np.linalg.pinv(self.TA_matrix)

        self.k_mat =  np.zeros((3,6))
        self.k_mat[0][0] = self.kp
        self.k_mat[1][1] = self.kp
        self.k_mat[2][2] = self.kp
        self.k_mat[0][3] = self.kd
        self.k_mat[1][4] = self.kd 
        self.k_mat[2][5] = self.kd
        
        self.error_mat = np.zeros((6,1))   # ex , ey , epsi , exr , eyr , epsir
        
        self.pwm_mat  =  np.zeros((4,1))   
        
        # self.pub_callback
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning
        self.subscriptiongoal = self.create_subscription(Float32MultiArray,'/mytopic/goal',self.guider_callback,10)
        self.subscriptiongoal  # prevent unused variable warning
    # def timer_callback(self):
    #     msg = Float64()
    #     msg.data = float(0) 
    #     print("Hello")
    #     self.publisher_.publish(msg)
    #     # self.get_logger().info(msg)

    def guider_callback(self,data):
        xg = data.data[0]
        yg = data.data[1]

        if(xg!=last_goal[0] and yg!=last_goal[1]  ):
            last_goal[0] = goal[0]
            last_goal[1] = goal[1]
        goal[0] = xg
        goal[1] = yg
        global xp
        global yp
        xt,yt = self.crosstrack(last_goal[0],last_goal[1],goal[0],goal[1],xp,yp)  # xt,yt are foot of the perpendicular in crosstrack
        a = math.atan(self.distance(xt,yt,xp,yp)/delta)
        pi_p = self.slope(last_goal[0],last_goal[1],goal[0],goal[1]) # Slope wrt global north
        psi_des = pi_p - a # Psi desired
        self.error_x = xt - xp # reducing crosstrack distance x cord
        self.error_y = yt - yp # reducing crosstrack distance y cord
        self.error_psi = psi_des - yaw # Allign with LOS vector , reduce error with psi desired
        dt = 0.1
        self.exr = self.error_x/dt
        self.eyr = self.error_y/dt
        self.epsir = self.error_psi/dt
        self.pwm_vec = np.dot(self.k_mat, self.error_mat)
        self.f_mat = np.dot(self.pseudo_TA_inv, self.pwm_vec)  
        for i in range(0,3):
            # self.pwm_mat[i] = 1400 + (200/(1+self.f_mat[i]))
            self.pwm_mat[i] = -100 + (200/(1+self.f_mat[i]))   # For wamv

        self.pwmlf = self.pwm_mat[0] # T1
        self.pwmrf = self.pwm_mat[1] # T2
        self.pwmla = self.pwm_mat[2] # T3
        self.pwmra = self.pwm_mat[3] # T4
        self.publisher_lf.publish(self.pwmlf)
        self.publisher_rf.publish(self.pwmrf)
        self.publisher_la.publish(self.pwmla)
        self.publisher_ra.publish(self.pwmra)
        
    # Flf  = ctrl_pwm[0]  # T1
    # Frf = ctrl_pwm[1] # T2
    # Fla = ctrl_pwm[2] # T4
    # Fra = ctrl_pwm[3] # T3


    def test(self,p, a, b):
        return (a*p+ b)
    

    def distance(self,x1,y1,x2,y2):
        d = math.sqrt(((x1-x2)**2) + ((y1-y2)**2))
        return d

    def crosstrack(self,a,b,c,d,e,f): # origin , goal , current_pos cordinates
        origin = [a,b]
        gol = [c,d]
        p = np.array([gol[0],origin[0]])
        q = np.array([gol[1],origin[1]])
        param, param_cov = curve_fit(self.test, p, q)
        p = e + param[0]*(abs(param[0]*e + param[1]-f)/(math.sqrt(param[0]*param[0] + param[1]*param[1])))
        q = f + param[1]*(abs(param[0]*e + param[1]-f)/(math.sqrt(param[0]*param[0] + param[1]*param[1])))
        return p,q

    def isInside(self,circle_x, circle_y, rad, p, q):  # For coa
        if ((p - circle_x) * (p - circle_x) +
            (q - circle_y) * (q - circle_y) <= rad * rad):
            return 1
        else:
            return 0
    
    def quaternion_to_euler(self,q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]
    
    def imu_callback(self,data):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = self.quaternion_to_euler(quaternion) # quaterion to euler angle conversion
        global yaw 
        yaw = euler[0] # yaw
        yawmsg = float(yaw)
        self.yawpubrad.publish(yawmsg)


    def gps_callback(self,data):
        global geo_lat
        geo_lat = float(data.latitude)  
        global geo_lon
        geo_lon = float(data.longitude)

        global origin_flag
        if origin_flag==0:
            origin[0] = data.latitude
            origin[1] = data.longitude
            last_goal[0] = 0
            last_goal[1] = 0
            goal[0] = 0
            goal[1] = 0
            origin_flag = 1

        azimuth,back_azimuth,distance = g.inv(origin[0],origin[1],geo_lat,geo_lon)
        azimuth = math.radians(azimuth)
        global xp # present position
        xp = math.sin(azimuth) * distance
        global yp 
        yp = math.cos(azimuth) * distance
        cord = [xp,yp]
        geomsg = Float32MultiArray(data = cord)
        self.geopub.publish(geomsg)

    # def gps_to_xy_ENU(self, origin_lat, origin_long, goal_lat, goal_long):
    # # Calculate distance and azimuth between GPS points
    #     geodesic = pyproj.Geod(ellps='WGS84')
    #     azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
    #     # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) 
    #     # by finding side lengths of a right-angle triangle
    #     # Convert azimuth to radians
    #     azimuth = radians(azimuth)
    #     # To get (x,y) in ENU frame
    #     self.y = adjacent = cos(azimuth) * distance
    #     self.x = opposite = sin(azimuth) * distance


    def slope(self,x1,y1,x2,y2): # Return slope from two points
        if(x1-x2==0):
            return math.pi/2
        else:
            m = math.atan(((y1-y2)/(x1-x2)))
            return math.atan(abs(m))

    # def pub_callback(self):
    #     msg = Float64
    #     msg.data = float(0)
    #     print("Hello")
    #     self.publisher_.publish(msg)

    # def gps_callback(self, msg):
    #     print("Hi")
    #     print(msg)
        # self.get_logger().info(float(msg.latitude))
        # self.get_logger().debug(msg.latitude)

    def guide(self):
        global xp
        global yp
        xt,yt = self.crosstrack(last_goal[0],last_goal[1],goal[0],goal[1],xp,yp)  # xt,yt are foot of the perpendicular in crosstrack
        a = math.atan(self.distance(xt,yt,xp,yp)/delta)
        pi_p = self.slope(last_goal[0],last_goal[1],goal[0],goal[1]) # Slope wrt global north
        psi_des = pi_p - a # Psi desired
        error_x = xt - xp # reducing crosstrack distance x cord
        error_y = yt - yp # reducing crosstrack distance y cord
        error_psi = psi_des - yaw # Allign with LOS vector , reduce error with psi desired
    
        Fx = pdc.update(error_x)
        Fy = pdc.update(error_y)
        Fn = pdc.update(error_psi)
        ctrl_pwm = np.dot(pdc.pseudo_TA_inv , np.array([Fx,Fy,Fn]))

        Flf  = ctrl_pwm[0]  # T1
        Frf = ctrl_pwm[1] # T2
        Fla = ctrl_pwm[2] # T4
        Fra = ctrl_pwm[3] # T3
        print(goal)
        print([xp,yp])
        print([Flf,Frf,Fla,Fra])
        
        # debugmsg = Flf
        # debug.publish(debugmsg)
        thrust_val[0] = int(pdc.inv_fin_func(Flf))
        thrust_val[1] = int(pdc.inv_fin_func(Frf))
        thrust_val[2] = int(pdc.inv_fin_func(Fra))
        thrust_val[3] = int(pdc.inv_fin_func(Fla))

        thrustmsg = Int32MultiArray(data=thrust_val)



        thrustpub.publish(thrustmsg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()


    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

