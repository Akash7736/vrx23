#!/usr/bin/env python3
import rospy
import numpy as np
import math
from scipy.optimize import curve_fit
from controller_pwm import PDController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import tf
import time
kp = float(input("Enter kp : "))
kd = float(input("Enter kd : "))
pdc = PDController(kp,kd)
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
 

def test(p, a, b):
    return (a*p+ b)


def distance(x1,y1,x2,y2):
    d = math.sqrt(((x1-x2)**2) + ((y1-y2)**2))
    return d

def crosstrack(a,b,c,d,e,f): # origin , goal , current_pos cordinates
    origin = [a,b]
    gol = [c,d]
    p = np.array([gol[0],origin[0]])
    q = np.array([gol[1],origin[1]])
    param, param_cov = curve_fit(test, p, q)
    p = e + param[0]*(abs(param[0]*e + param[1]-f)/(math.sqrt(param[0]*param[0] + param[1]*param[1])))
    q = f + param[1]*(abs(param[0]*e + param[1]-f)/(math.sqrt(param[0]*param[0] + param[1]*param[1])))
    return p,q

def isInside(circle_x, circle_y, rad, p, q):  # For coa
    if ((p - circle_x) * (p - circle_x) +
        (q - circle_y) * (q - circle_y) <= rad * rad):
        return 1
    else:
        return 0
    
def imu_callback(data):
    quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion) # quaterion to euler angle conversion
    global yaw 
    yaw = euler[2] # yaw
    yawmsg = float(yaw)
    yawpubrad.publish(yawmsg)


def vo_callback(data):
    global vo_x
    vo_x = float(data.pose.position.x)  
    global vo_y
    vo_y = float(data.pose.position.y)

    global origin_flag
    if origin_flag==0:
        origin[0] = vo_x
        origin[1] = vo_y
        last_goal[0] = origin[0]
        last_goal[1] = origin[1]
        goal[0] = last_goal[0]
        goal[1] = last_goal[1]
        origin_flag = 1



    cord = [vo_x,vo_y]
    global xp # present position
    xp = cord[0]
    global yp 
    yp = cord[1]
    vomsg = Float32MultiArray(data = cord)
    vopub.publish(vomsg)


def slope(x1,y1,x2,y2): # Return slope from two points
    if(x1-x2==0):
        return math.pi/2
    else:
        m = math.atan(((y1-y2)/(x1-x2)))
        return math.atan(abs(m))

def stop():
    msg = Int32MultiArray(data = [1500,1500,1500,1500])
    thrustpub.publish(msg)

def guide():
    global xp
    global yp
    xt,yt = crosstrack(last_goal[0],last_goal[1],goal[0],goal[1],xp,yp)  # xt,yt are foot of the perpendicular in crosstrack
    a = math.atan(distance(xt,yt,xp,yp)/delta)
    pi_p = slope(last_goal[0],last_goal[1],goal[0],goal[1]) # Slope wrt global north
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
    
    debugmsg = Flf
    debug.publish(debugmsg)
    thrust_val[0] = int(pdc.inv_fin_func(Flf))
    thrust_val[1] = int(pdc.inv_fin_func(Frf))
    thrust_val[2] = int(pdc.inv_fin_func(Fra))
    thrust_val[3] = int(pdc.inv_fin_func(Fla))

    thrustmsg = Int32MultiArray(data=thrust_val)



    thrustpub.publish(thrustmsg)



def main():
    if(isInside(goal[0],goal[1],coa,xp,yp)) == 1:
        print("Goal Reached !")
        stop()
       

if __name__ == '__main__':
    rospy.init_node('guider',anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            
            vopub = rospy.Publisher('/vo/cord', Float32MultiArray, queue_size=10)
            yawpubrad =  rospy.Publisher('/imu/yaw', Float32, queue_size=10)
            thrustpub = rospy.Publisher('/thrust_val', Int32MultiArray, queue_size=10) 
            debug = rospy.Publisher('/debug_val', Float32, queue_size=10) 

            # rospy.loginfo("Waiting for VO data")
            # rospy.wait_for_message('/zed2i/zed_node/pose', PoseStamped, timeout=10)  
            # rospy.loginfo("Receiving VO Data.....")      

            rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, vo_callback)
            rospy.Subscriber("/guider/goal", Float32MultiArray, guider_callback)
            rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)

            guide()
            main()
            
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

  
