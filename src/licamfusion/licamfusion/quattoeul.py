from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

import numpy as np


def rot_x(theta): # roll
    R = np.array([[1,0,0],
         [0,np.cos(theta),-np.sin(theta)],
         [0,np.sin(theta),np.cos(theta)]])
    return R

def rot_y(theta): # pitch
    R = np.array([[np.cos(theta),0,np.sin(theta)],
         [0,1,0],
         [-np.sin(theta),0,np.cos(theta)]])
    return R

def rot_z(theta):  #yaw
    R = np.array([[np.cos(theta),-np.sin(theta),0],
         [np.sin(theta),np.cos(theta),0],
          [0,0,1],])
    return R

def tf2tfmat(msg:TransformStamped):
    r,p,y = euler_from_quaternion([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w])
    rotmat = rot_z(y)@rot_y(p)@rot_x(r)
    transmat = np.array([[msg.transform.translation.x],[msg.transform.translation.y],[msg.transform.translation.z]])
    tfmat = np.concatenate((rotmat,transmat),axis=1)
    return tfmat

# print(euler_from_quaternion([0,0.13052599997931216,0,0.991444886682765]))
# print(rot_z(0)@rot_y(0.261799)@rot_x(0))
# print(quat2rot([0,0.13052599997931216,0,0.991444886682765]))
