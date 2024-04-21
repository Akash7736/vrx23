import yaml
import numpy as np
import math
from tf_transformations import euler_from_quaternion
import pyproj
# with open('pid.yml', 'r') as file:
#     prime_service = yaml.safe_load(file)

# print(prime_service['Controller']['kp22'])
# asd = None
# print(asd)
# def check():
#     return 
# print(check())  # This will return None

# print(np.linalg.norm([2,3]))

goal = [-34.424223000000,150.479736000000,0]
origin = [-33.72277565710174,150.673991252778]
# g = Geod(ellps = 'WGS84')
# def gps2enu(orlon, orlat,lon,lat):
#     azimuth,back_azimuth,distance = g.inv(orlon, orlat,lon,lat)
#     azimuth = math.radians(azimuth) 
#     x = math.sin(azimuth) * distance   
#     y = math.cos(azimuth) * distance
#     print(azimuth)
#     return x,y
# print(gps2enu(origin[0],origin[1],goal[0],goal[1]))

#****************************************************************************88
    # def inv(self, lons1, lats1, lons2, lats2, radians=False):
    #     """
    #     inverse transformation - Returns forward and back azimuths, plus
    #     distances between initial points (specified by lons1, lats1) and
    #     terminus points (specified by lons2, lats2).

    #     Works with numpy and regular python array objects, python
    #     sequences and scalars.

    #     if radians=True, lons/lats and azimuths are radians instead of
    #     degrees. Distances are in meters.
    #     """
    #     # process inputs, making copies that support buffer API.
    #     inx, xisfloat, xislist, xistuple = _copytobuffer(lons1)
    #     iny, yisfloat, yislist, yistuple = _copytobuffer(lats1)
    #     inz, zisfloat, zislist, zistuple = _copytobuffer(lons2)
    #     ind, disfloat, dislist, distuple = _copytobuffer(lats2)
    #     super(Geod, self)._inv(inx, iny, inz, ind, radians=radians)
    #     # if inputs were lists, tuples or floats, convert back.
    #     outx = _convertback(xisfloat, xislist, xistuple, inx)
    #     outy = _convertback(yisfloat, yislist, xistuple, iny)
    #     outz = _convertback(zisfloat, zislist, zistuple, inz)
    #     return outx, outy, outz
#*********************************************************************************
# def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
# # Calculate distance and azimuth between GPS points
#     geodesic = pyproj.Geod(ellps='WGS84')
#     azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
# # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
# # Convert azimuth to radians
#     azimuth = math.radians(azimuth)
#     y = adjacent = math.cos(azimuth) * distance
#     x = opposite = math.sin(azimuth) * distance
#     print(x)
#     print(y)
#     return x, y


# calc_goal(origin[0],origin[1],goal[0],goal[1])

#************************************************************************************

        
# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (w in last place) to euler roll, pitch, yaw
#     quaternion = [x, y, z, w]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     x = quaternion[0]
#     y = quaternion[1]
#     z = quaternion[2]
#     w = quaternion[3]

#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     sinp = 2 * (w * y - z * x)
#     pitch = np.arcsin(sinp)

#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw
# print(euler_from_quaternion([0,0,-0.479425538604203,0.8775825618903728]))

# ---
# header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# pose:
#   position:
#     x: -33.7226643
#     y: 150.673947
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: -0.479425538604203
#     w: 0.8775825618903728
# **************************************************************************************************
# def change_range(angle):
#     if -2*pi<=angle<=-pi:
#         return angle+2*pi
#     elif pi<angle<=2*pi:
#         return angle-2*pi
#     else:
#         return angle

# #****************************************************************************
# TA_matrix = np.array([[0.707, 0.707, 0.707, 0.707],
#                                     [-0.707, 0.707, 0.707, -0.707],
#                                     [-1.626,  1.626,  -2.333,  2.333]])

# squarer = lambda t: math.exp(t)
# x = np.array([1, 2, 3, 4, 5])
# print(squarer(TA_matrix))

arr = np.array([[222],[333],[5555]])
print(np.exp(-arr))