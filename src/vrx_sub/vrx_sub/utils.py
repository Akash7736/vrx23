#!/usr/bin/env python3
import numpy as np
from sensor_msgs.msg import NavSatFix
from vrx_sub.config import DATUM, GEODESIC, GLF_MODE
from geometry_msgs.msg import Pose
import time
import rclpy.time
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import message_filters
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class SyncSubscription:
    def __init__(self, node: Node, df, callback, queue_size=10, approx_time=0) -> None:
        if approx_time == 0:
            node.get_logger().debug(f'Subs to topics in sync : {",".join(df.keys())}')
            self.msgs_sub = message_filters.TimeSynchronizer(
                [message_filters.Subscriber(node, df[topic], topic) for topic in df], queue_size=queue_size)
        else:
            self.msgs_sub = message_filters.ApproximateTimeSynchronizer(
                [message_filters.Subscriber(node, df[topic], topic) for topic in df], slop=approx_time, queue_size=queue_size)
        self.msgs_sub.registerCallback(callback)


class TF_Frames:
    def __init__(self, node: Node) -> None:
        self.buffer = Buffer()
        self.l = TransformListener(self.buffer, node)

    def get_tf(self, from_frame_rel, to_frame_rel, stamp):
        try:
            t = self.buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                stamp)
            trans_mat = np.array(
                [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            # r,p,y = euler_from_quaternion(t.transform.rotation)
            # rot_mat = rotation_matrix_from_euler(r, p, y)
            rot_mat = quaternion_to_rotation_matrix(t.transform.rotation)
            return trans_mat, rot_mat
        except Exception as e:
            print(e)
            return None

    def translate_rotate(self, points, from_frame, to_frame):
        tf = self.get_tf(from_frame, to_frame)
        if tf is None:
            return False, points
        else:
            return True, (tf[1] @ (points + tf[0]).T).T

    def rotate_translate(self, points, from_frame, to_frame, stamp = rclpy.time.Time()):
        tf = self.get_tf(from_frame, to_frame, stamp)
        if tf is None:
            return False, points
        else:
            print(tf)
            return True, np.dot(tf[1], points.T).T + tf[0]

def median(x):
    m,n = x.shape
    middle = np.arange((m-1)>>1,(m>>1)+1)
    x = np.partition(x,middle,axis=0)
    return x[middle].mean(axis=0)

def remove_outliers_mask(data,thresh=2.0):           
    m = median(data)                            
    s = np.abs(data-m)                          
    return (s<median(s)*thresh).all(axis=1)


def timeTaken(fxn, *kwargs):
    b = time.time()
    v = fxn(*kwargs)
    e = time.time()

    print(str(fxn.__name__), "->", e - b)

    return v


def inv_glf(T):
    if not GLF_MODE:
        return T
    if T > 0:
        A, K, B, v, C, M = 0.01, 59.82, 5.00, 0.38, 0.56, 0.28
    else:
        A, K, B, v, C, M = -199.13, -0.09, 8.84, 5.34, 0.99, -0.57
    # try:
    return np.abs((M - (np.log(((K-A)/(T-A))**v - C))/B) * T) / T
    # except:
    #     return 0.0

def pose_to_numpy(pose):
    rot_mat = quaternion_to_rotation_matrix(pose.orientation)
    trans = np.array([pose.position.x,
                         pose.position.y, pose.position.z])
    return trans, rot_mat

def voxel_filter(voxel_mat, points):
    _, idx = np.unique(np.int32(points * voxel_mat), return_index=True, axis=0)
    return idx

# def clustering_dbscan(eps = 0.01, min_sample = 1):


def XYZ_to_UV(K, points: np.ndarray):
    trans_points = np.zeros((len(points), 3))
    trans_points[:, 2] = points[:, 0]
    trans_points[:, 0] = points[:, 1] * -1
    trans_points[:, 1] = points[:, 2] * -1

    pixels = np.dot(K, trans_points.T).T
    pixels = np.int32(pixels[:, :2] / pixels[:, 2].reshape(-1, 1))

    return pixels


def rotation_matrix_from_euler(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    return np.dot(R_z, np.dot(R_y, R_x.T))


def transform_frame(tf, points):
    (trans, rot) = tf
    return (rot @ (points).T).T + trans


def transfrom_frame_2d(tf, points):
    (trans, rot) = tf
    return rot[:2, :2] @ (points + trans[:2])


def laser2numpy(data, max_range, min_range):

    angel_min = data.angle_min
    angel_inc = data.angle_increment
    ranges = np.array(data.ranges)
    angles = angel_min + np.arange(ranges.shape[0]) * angel_inc
    mask = (ranges < max_range) & (ranges > min_range)
    angles = angles[mask]
    ranges = ranges[mask]
    points = np.hstack([np.cos(angles) * ranges, np.sin(angles) * ranges, 0])
    return points


def global_to_local_rot(theta):
    return np.array(
        [
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def change_range(angle):
    if -2 * np.pi <= angle <= -np.pi:
        return angle + 2 * np.pi
    elif np.pi < angle <= 2 * np.pi:
        return angle - 2 * np.pi
    else:
        return angle


def quaternion_to_rotation_matrix(x, y, z, w):

    rotation_matrix = np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y -
                2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x -
                2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 *
                x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )

    return rotation_matrix


def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    rotation_matrix = np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y -
                2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x -
                2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 *
                x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )
    return rotation_matrix


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def gps_pose_to_pose(gps: Pose, datum=DATUM):
    azimuth, _, distance = GEODESIC.inv(
        datum[1], datum[0], gps.position.y, gps.position.x)
    azimuth = np.radians(azimuth)
    gps.position.x, gps.position.y = np.sin(
        azimuth) * distance, np.cos(azimuth) * distance
    return gps


def gps_to_xy_ENU(gps_msg: NavSatFix, datum=DATUM):

    azimuth, _, distance = GEODESIC.inv(
        datum[1], datum[0], gps_msg.longitude, gps_msg.latitude)
    azimuth = np.radians(azimuth)
    return np.array([np.sin(azimuth) * distance, np.cos(azimuth) * distance, gps_msg.altitude])
