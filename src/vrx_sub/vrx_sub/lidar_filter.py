#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry, OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_msgs.msg import Header
from utils import SyncSubscription, pose_to_numpy, XYZ_to_UV, remove_outliers
from config import LIDAR_WAMV_LINK_TO_BASE_LINK_TF, LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF, FRONT_LEFT_K


class Lidar(Node):
    def __init__(self, model_path, max_range, min_range):
        super().__init__('Lidar')
        sensor_df = {'/wamv/sensors/lidars/lidar_wamv_sensor/points': PointCloud2,
                     '/wamv/kf/state': Odometry}

        self.bridge = CvBridge()

        self.create_subscription(
            Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.img_callback)

        self.sensor_sub = SyncSubscription(
            self, sensor_df, self.sensor_callback)

        self.model = YOLO()

        self.pc = PointCloud2()
        self.pose = Pose()
        self.image_msg = Image()
        self.pc_pub = self.create_publisher(
            PointCloud2, '/wamv/sensors/clustered_lidar/points', 10)
        self.oc_pub = self.create_publisher(OccupancyGrid, '/map_grid', 10)
        self.grid_width = 120
        self.grid_height = 120
        self.grid_res = 0.5
        self.occup_grid = np.zeros((self.grid_width, self.grid_height))

        self.loop = self.create_timer(
            timer_period_sec=0.1, callback=self.compute)

        self.yolo = self.create_timer(
            timer_period_sec=0.1, callback=self.image_compute)

        self.max_range = max_range
        self.min_range = min_range
        self.baselink_trans, self.baselink_rot = LIDAR_WAMV_LINK_TO_BASE_LINK_TF
        self.lidar_cam_trans, self.lidar_cam_rot = LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF

        self.voxel_mat = np.array([0.1, 0.1, 0.1])

        self.prev_pose = Pose()
        self.prev_base_cloud = PointCloud2()
        self.delay_lidar_cam = 0.02

        self.cloud = None
        self.triggered_sensor = False
        self.triggered_image = False

    def img_callback(self, img: Image):
        self.image = img
        self.triggered_image = True

    def sensor_callback(self, pc: PointCloud2, pose: Odometry):
        self.pc = pc
        self.pose = pose.pose.pose
        self.triggered_sensor = True

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

    def image_compute(self):
        if not (self.triggered_image and self.triggered_sensor):
            return

        delay_lidar_cam = abs(self.pc.header.stamp.nanosec -
                              self.image_msg.header.stamp.nanosec) * 1e-9

        if not delay_lidar_cam < self.delay_lidar_cam:
            return
        if self.cloud is None:
            return

        image = self.bridge.imgmsg_to_cv2(self.image_msg, 'rbg8')
        results = self.model(image)
        cam_cloud = (self.lidar_cam_rot.T @ self.cloud.T).T - \
            self.lidar_cam_trans

        base_link_cloud = (self.baselink_rot.T @
                           self.cloud.T).T + (-self.baselink_trans)
        
        pixels = XYZ_to_UV(FRONT_LEFT_K, cam_cloud)
        detected_objects = []
        for infer in results:
            for xyxy, cls, conf in zip(infer.boxes.xyxy, infer.boxes.cls, infer.boxes.conf):
                conf = round(float(conf.cpu().numpy()), 3)
                pixs = xyxy.cpu().numpy()
                l = int(cls.cpu().numpy())
                p1, p2 = np.int32(pixs[:2]), np.int32(pixs[-2:])
                mask = (pixels > p1).all(axis=1) & (pixels < p2).all(axis=1)
                bb_cloud = base_link_cloud[mask]
                if bb_cloud.shape[0] == 0:
                    continue
                bb_cloud = remove_outliers(bb_cloud)
                center = np.mean(bb_cloud, axis=0)
                detected_objects.append(())
                

    def compute(self):
        if not self.triggered_sensor:
            return

        self.triggered_sensor = False
        self.triggered_image = False

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
        theta = np.arctan2(cloud_numpy[:, 1], cloud_numpy[:, 0])
        theta_mask = (theta < np.pi/3) & (theta > -np.pi/3)
        cloud_numpy = cloud_numpy[mask]
        self.cloud = cloud_numpy[theta_mask]

        base_link_cloud = (self.baselink_rot.T @
                           cloud_numpy.T).T + (-self.baselink_trans)

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
            header=header, fields=self.pc.fields[:3], points=base_link_cloud)

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


def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar(model_path='', max_range=120, min_range=0.85)
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
