import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np
import math

from pyproj import Geod
from scipy.optimize import curve_fit
from controller_wamv import PDController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import tf
import time

pdc = PDController(0.1,0.2)
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
        # self.pub_callback
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriptiongps = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscriptiongps  # prevent unused variable warning
        self.subscriptionimu = self.create_subscription(Imu,'/wamv/sensors/imu/imu/data',self.imu_callback,10)
        self.subscriptionimu  # prevent unused variable warning

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
    
     
    def imu_callback(self,data):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        global yaw 
        yaw = euler[2] # yaw
        yawmsg = float(yaw)
        self.yawpubrad.publish(yawmsg)


    def gps_callback(self,data):
        global geo_x
        geo_x = float(data.latitude)  
        global geo_y
        geo_y = float(data.longitude)

        global origin_flag
        if origin_flag==0:
            origin[0] = data.latitude
            origin[1] = data.longitude
            last_goal[0] = 0
            last_goal[1] = 0
            goal[0] = 0
            goal[1] = 0
            origin_flag = 1

        cord = g.inv(origin[0],origin[1],geo_x,geo_y)
        global xp # present position
        xp = cord[0]
        global yp 
        yp = cord[1]
        geomsg = Float32MultiArray(data = cord)
        self.geopub.publish(geomsg)


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