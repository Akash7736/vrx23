#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np
from std_msgs.msg import Header
from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter

from utils import SyncSubscription, pose_to_numpy, XYZ_to_UV, remove_outliers_mask, timeTaken
from config import LIDAR_WAMV_LINK_TO_BASE_LINK_TF, LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF, FRONT_LEFT_K


class Map(Node):
    def __init__(self, max_range, min_range):
        super().__init__('Map')
        sensor_df = {'/wamv/sensors/lidars/lidar_wamv_sensor/points': PointCloud2,
                     '/wamv/kf/state': Odometry,
                     '/wamv/yolo/results': ParamVec,
                     }

        self.sensor_sub = SyncSubscription(
            self, sensor_df, self.sensor_callback, approx_time=0.1)

        self.pc = PointCloud2()
        self.pose = Pose()
        self.dets = ParamVec()

        self.pc_pub = self.create_publisher(
            PointCloud2, '/wamv/sensors/clustered_lidar/points', 10)
        self.oc_pub = self.create_publisher(OccupancyGrid, '/map_grid', 10)

        self.det_pub = self.create_publisher(ParamVec,'/wamv/detecion/localization', 10)

        self.grid_width = 120
        self.grid_height = 120
        self.grid_res = 0.5
        self.occup_grid = np.zeros((self.grid_width, self.grid_height))

        self.loop = self.create_timer(
            timer_period_sec=0.1, callback=self.compute)

        self.max_range = max_range
        self.min_range = min_range
        self.baselink_trans, self.baselink_rot = LIDAR_WAMV_LINK_TO_BASE_LINK_TF
        self.lidar_cam_trans, self.lidar_cam_rot = LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF

        self.voxel_mat = np.array([0.1, 0.1, 0.1])

        self.prev_pose = Pose()
        self.prev_base_cloud = PointCloud2()

        self.triggered = False

    def sensor_callback(self, pc: PointCloud2, pose: Odometry, dets: ParamVec):
        self.pc = pc
        self.pose = pose.pose.pose
        self.dets = dets
        self.triggered = True

    def cloud_to_grid(self, cloud2d, width, height, res):
        mask = (cloud2d[:, 0] < width*res/2) & (cloud2d[:, 1] < height*res /
                                                2) & (cloud2d[:, 0] > -width*res/2) & (cloud2d[:, 1] > -height*res/2)
        cloud2d = cloud2d[mask]
        trans_cloud2d = cloud2d + np.array([width*res/2, height*res/2])
        grid_idxs = np.floor(trans_cloud2d / res)
        grid_idx = np.unique(grid_idxs, axis=0).astype(int)
        grid = np.zeros((width, height))
        grid[grid_idx[:, 1], grid_idx[:, 0]] = 20
        return grid

    def grid_to_cloud(self, grid, width, height, res):
        grid_idx = np.array(np.where(grid > 0))[::-1].T + 0.5
        if grid_idx.shape[0] == 0:
            return np.array([[0, 0]])
        grid_idxs = grid_idx * res
        trans_cloud2d = grid_idxs - np.array([width*res/2, height*res/2])
        return trans_cloud2d
    
    # def compute(self):
    #     timeTaken(self.compute0)
    def compute(self):
        if not self.triggered:
            return
        
        self.get_logger().info('Computing......')

        self.triggered = False

        if self.prev_pose is None:
            self.prev_pose = self.pose

        cloud = pc2.read_points(self.pc)
        trans, rot_mat = pose_to_numpy(self.pose)
        prev_trans, prev_rot_mat = pose_to_numpy(self.prev_pose)

        cloud_numpy = np.zeros((cloud.shape[0], 3))
        cloud_numpy[:, 0] = cloud['x']
        cloud_numpy[:, 1] = cloud['y']
        cloud_numpy[:, 2] = cloud['z']

        dist = np.linalg.norm(cloud_numpy[:, :2], axis=1)
        mask = (dist < self.max_range) & (dist > self.min_range)
        cloud_numpy = cloud_numpy[mask]

        theta = np.arctan2(cloud_numpy[:, 1], cloud_numpy[:, 0])
        theta_mask = (theta < np.pi/3) & (theta > -np.pi/3)

        base_link_cloud = (self.baselink_rot.T @
                           cloud_numpy.T).T + (-self.baselink_trans)
        
        cam_cloud = (self.lidar_cam_rot.T @
                     cloud_numpy[theta_mask].T).T - self.lidar_cam_trans
        pixels = XYZ_to_UV(FRONT_LEFT_K, cam_cloud)
        cam_cloud = base_link_cloud[theta_mask]
        labels = np.ones(cam_cloud.shape[0]) * -1

        for param in self.dets.params:
            param: Parameter
            l = param.value.integer_value
            conf = param.value.double_value
            pixs = param.value.integer_array_value
            p1, p2 = np.int32(pixs[:2]), np.int32(pixs[-2:])
            mask = (pixels > p1).all(axis=1) & (pixels < p2).all(axis=1)
            bb_cloud = cam_cloud[mask]

            if bb_cloud.shape[0] == 0:
                return
            out_mask = remove_outliers_mask(bb_cloud)
            bb_cloud = bb_cloud[out_mask]
            coord = np.mean(bb_cloud, axis = 0).tolist()
            param.value.double_array_value = coord
            net_mask = mask.copy()
            net_mask[mask] = out_mask
            labels[net_mask] = l
            

        cam_cloud_mask = labels == -1
        cam_cloud = cam_cloud[~cam_cloud_mask]
        labels = labels[~cam_cloud_mask].reshape(-1,1)
        clustered_cam_cloud = np.hstack(
            [cam_cloud, labels])

        prev_base_cloud = self.grid_to_cloud(
            self.occup_grid, self.grid_width, self.grid_height, self.grid_res)

        tf_prev_cloud = (rot_mat[:2, :2].T @ prev_rot_mat[:2, :2]
                         @ prev_base_cloud.T).T + prev_trans[:2] - trans[:2]

        odom_cloud = (rot_mat @ base_link_cloud.T).T + trans
        z_mask = (odom_cloud[:, 2] < 3.0) & (odom_cloud[:, 2] > 0.35)
        base_link_cloud = base_link_cloud[z_mask]

        curr_grid = self.cloud_to_grid(
            base_link_cloud[:, :2], self.grid_width, self.grid_height, self.grid_res)
        prev_grid = self.cloud_to_grid(
            tf_prev_cloud, self.grid_width, self.grid_height, self.grid_res)

        self.occup_grid = curr_grid + prev_grid
        self.prev_pose = self.pose

        header = Header()
        header.stamp = self.pc.header.stamp
        header.frame_id = 'wamv/wamv/base_link'
        out_msg = pc2.create_cloud(
            header=header, fields=self.pc.fields[:4], points=clustered_cam_cloud)

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.pc.header.stamp
        grid_msg.header.frame_id = '/wamv/wamv/base_link'
        grid_msg.info.map_load_time = self.pc.header.stamp
        grid_msg.info.resolution = self.grid_res
        grid_msg.info.height, grid_msg.info.width = self.grid_height, self.grid_width
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = -self.grid_width * self.grid_res / 2
        grid_msg.info.origin.position.y = -self.grid_height * self.grid_res / 2

        grid_msg.data = self.occup_grid.astype(int).ravel().tolist()
        self.pc_pub.publish(out_msg)
        self.oc_pub.publish(grid_msg)
        self.det_pub.publish(self.dets)


def main(args=None):
    rclpy.init(args=args)
    map = Map(max_range=60, min_range=0.85)
    rclpy.spin(map)
    map.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
