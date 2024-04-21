import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np
import math


from controller_wamv import PDController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
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

def guider_callback(data):
    
    xg = data.data[0]
    yg = data.data[1]
    

    if(xg!=last_goal[0] and yg!=last_goal[1]  ):
        last_goal[0] = goal[0]
        last_goal[1] = goal[1]
        
    goal[0] = xg
    goal[1] = yg

