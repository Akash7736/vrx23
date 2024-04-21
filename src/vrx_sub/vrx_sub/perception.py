#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
import rclpy.time
from ultralytics import YOLO
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from utils import XYZ_to_UV, transform_frame, quaternion_to_rotation_matrix, voxel_filter, TF_Frames, SyncSubscription
import numpy as np
from cv_bridge import CvBridge
from config import LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF, FRONT_LEFT_K, LIDAR_WAMV_LINK_TO_BASE_LINK_TF
import sensor_msgs_py.point_cloud2 as pc2
import cv2

from nav_msgs.msg import OccupancyGrid

'''
Code is lite
'''


class Perception(Node):
    def __init__(self, model_path):
        super().__init__('Perception')
        # self.model = YOLO(model_path)
        # self.model.eval()
        self.bridge = CvBridge()

        sensor_struct = {
            '/wamv/sensors/lidars/lidar_wamv_sensor/points': PointCloud2,
            '/wamv/sensors/imu/imu/data': Imu
        }
        self.imu_lidar = SyncSubscription(
            self, sensor_struct, self.imu_lidar_callback, 10)
        self.cam_sub = self.create_subscription(
            Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.img_callback, 10)
        # self.lidar_sub = self.create_subscription(
            # PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        self.kf_sub = self.create_subscription(
            Odometry, '/wamv/kf/state', self.odom_callback, 10)
        self.pc_pub = self.create_publisher(
            PointCloud2, '/wamv/sensors/clustered_lidar/points', 10)
        self.tf_frame = TF_Frames(self)
        self.K = FRONT_LEFT_K
        self.delay_lidar_cam_th = 0.02
        self.voxel_mat = np.array([
            [7, 7, 10],
        ])
        self.loop_fusion = self.create_timer(
            timer_period_sec=0.1, callback=self.fusion)

        self.loop_yolo = self.create_timer(
            timer_period_sec=0.1, callback=self.yolo_detect)

        self.loop_cluster = self.create_timer(
            timer_period_sec=0.1, callback=self.cluster)

        self.pose = None
        self.image = None
        self.pc_laser = None
        self.yolo_results = None
        self.cloud = None
        self.orientaion = None

        self.lidar_msg_ping = False
        self.camera_msg_ping = False
        self.odom_msg_ping = False
        self.yolo_ping = False
        self.cluster_ping = False
        self.imu_msg_ping = False

        self.baselink_trans, self.baselink_rot = LIDAR_WAMV_LINK_TO_BASE_LINK_TF

    def imu_lidar_callback(self, pc: PointCloud2, imu: Imu):
        self.pc_laser = pc
        self.orientaion = imu.orientation
        self.imu_msg_ping = True
        self.lidar_msg_ping = True

    def img_callback(self, img: Image):
        self.image = img
        self.camera_msg_ping = True

    def odom_callback(self, msg: Odometry):
        if self.pose is None:
            self.pose = PoseStamped()
        self.pose.header = msg.header
        self.pose.pose = msg.pose.pose
        self.odom_msg_ping = True

    def cluster(self):
        if not (self.lidar_msg_ping and self.imu_msg_ping and self.camera_msg_ping):
            return
        self.lidar_msg_ping, self.imu_msg_ping, self.camera_msg_ping = False, False, False

        rot_mat = quaternion_to_rotation_matrix(self.orientaion)
        cloud = pc2.read_points(self.pc_laser)
        cloud_numpy = np.zeros((cloud.shape[0], 4))
        cloud_numpy[:, 0] = cloud['x']
        cloud_numpy[:, 1] = cloud['y']
        cloud_numpy[:, 2] = cloud['z']

        dist = np.linalg.norm(cloud_numpy, axis=1)
        cloud_numpy = cloud_numpy[(dist < 50.0) & (dist > 0.75)]
        idx = voxel_filter(self.voxel_mat, cloud_numpy[:, :3])
        cloud_numpy = cloud_numpy[idx]
        base_link_cloud = transform_frame(
            (- self.baselink_trans, self.baselink_rot.T), points=cloud_numpy[:, :3])
        base_cloud = np.dot(rot_mat, base_link_cloud.T).T

        z_mask = (base_cloud[:, 2] > 0.35) & (base_cloud[:, 2] < 3.0)
        cloud_numpy = cloud_numpy[z_mask]

        model = DBSCAN(eps=0.01, min_samples=2)
        scaled_pc = StandardScaler().fit_transform(cloud_numpy[:,:3])
        model.fit(scaled_pc)
        mask = (model.labels_ == -1)
        cloud_numpy = cloud_numpy[~mask]
        cloud_numpy[:, 3] = model.labels_[~mask] / 10

        header = Header()
        header.stamp = self.pc_laser.header.stamp
        header.frame_id = 'wamv/wamv/lidar_wamv_link'
        out_msg = pc2.create_cloud(
            header=header, fields=self.pc_laser.fields[:4], points=cloud_numpy)
        self.pc_pub.publish(out_msg)

    def cluster0(self):
        if not (self.lidar_msg_ping and self.odom_msg_ping and self.camera_msg_ping):
            return
        self.lidar_msg_ping, self.odom_msg_ping, self.camera_msg_ping = False, False, False
        # r, p, yaw = euler_from_quaternion(self.pose.pose.orientation)
        imu_rot_mat = quaternion_to_rotation_matrix(self.pose.pose.orientation)
        trans = np.array([self.pose.pose.position.x,
                         self.pose.pose.position.y, self.pose.pose.position.z])
        cv_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cloud = pc2.read_points(self.pc_laser)
        cloud_numpy = np.zeros((cloud.shape[0], 4))
        cloud_numpy[:, 0] = cloud['x']
        cloud_numpy[:, 1] = cloud['y']
        cloud_numpy[:, 2] = cloud['z']

        
        # z_mask = (cloud_numpy[:, 2] < 5.0) & (cloud_numpy[:, 2] > -2.0)
        # cloud_numpy = cloud_numpy[z_mask]
        dist = np.linalg.norm(cloud_numpy, axis=1)
        cloud_numpy = cloud_numpy[(dist < 50.0)]
        # print(self.tf_frame.tf_frame)
        sucess, cloud_numpy[:, :3] = self.tf_frame.rotate_translate(
            points=cloud_numpy[:, :3], to_frame='wamv/wamv/odom', from_frame='wamv/wamv/lidar_wamv_link', stamp=self.pose.header.stamp)
        if not sucess:
            self.get_logger().error('TF Failed')
            return

        # sucess, cloud_numpy[:, :3] = self.tf_frame.rotate_translate(
        #     points=cloud_numpy[:, :3], from_frame='wamv/wamv/odom', to_frame='wamv/wamv/base_link')
        # if not sucess:
        #     self.get_logger().error('TF Failed')
        #     return

        # index = voxel_filter(voxel_mat=self.voxel_mat, points=cloud_numpy[:, :2])
        # cloud_numpy = cloud_numpy[index]
        # print(cloud_numpy[:, 2])
        # theta = np.arctan2(cloud_numpy[:, 1], cloud_numpy[:, 0])
        # theta_mask = (theta > -np.pi/3) & (theta < np.pi/3)
        # cloud_numpy = cloud_numpy[theta_mask]

        # cloud_numpy[:, :3] = transform_frame(
        #     (- self.baselink_trans, self.baselink_rot.T), points=cloud_numpy[:, :3])
        # cloud_numpy[:, :3] = transform_frame(
        #     (trans, imu_rot_mat), points=cloud_numpy[:, :3])
        # pixs = XYZ_to_UV(self.K, cloud_numpy[:, :3])
        # pixs_mask = (pixs[:, 1] < cv_image.shape[0]) & (
        #         pixs[:, 0] < cv_image.shape[1]) & (pixs[:, 0] > 0) & (pixs[:, 1] > 0)
        # pixs = pixs[pixs_mask]
        # cloud_numpy = cloud_numpy[pixs_mask]
        # color_cloud = (cv_image[pixs[:, 1], pixs[:, 0]])
        # pc = np.hstack([cloud_numpy[:, :3], color_cloud])
        # print(pc.shape)
        # scaled_pc = StandardScaler().fit_transform((imu_rot_mat @ cloud_numpy[:, :3].T).T)
        # _,idx = np.unique(scaled_pc[:, 2], return_index=True)
        # print(idx)
        # cloud_numpy = cloud_numpy[idx]
        # scaled_pc = scaled_pc[idx]
        # print(scaled_pc.shape)
        model = DBSCAN(eps=0.2, min_samples=2)
        model.fit(cloud_numpy[:, :3])
        mask = (model.labels_ == -1)
        cloud_numpy = cloud_numpy[~mask]
        self.labels = model.labels_[~mask]
        self.cloud = cloud_numpy[:, :3]

        # cloud_numpy[:, :3] = (imu_rot_mat @ cloud_numpy[:, :3].T).T + trans
        # / len(np.unique(self.labels))

        # idx = np.random.randint(low = 0, high = np.max(self.labels), size = 1)
        # mask = self.labels == idx
        # cloud_numpy = cloud_numpy[mask]
        cloud_numpy[:, 3] = self.labels / 10
        header = Header()
        header.stamp = self.pc_laser.header.stamp
        header.frame_id = 'wamv/wamv/odom'
        # cloud_numpy[:, :3] = (self.imu_rot_mat @ cloud_numpy[:, :3].T).T

        out_msg = pc2.create_cloud(
            header=header, fields=self.pc_laser.fields[:4], points=cloud_numpy)
        self.pc_pub.publish(out_msg)

    def yolo_detect(self):
        if not self.camera_msg_ping:
            return

    def fusion(self):
        if not (self.camera_msg_ping and self.lidar_msg_ping):
            return
        return
        self.triggers = np.array([False, False, False])
        delay_lidar_cam = abs(
            self.pc_laser.header.stamp.nanosec - self.image.header.stamp.nanosec) * 1e-9
        if not delay_lidar_cam < self.delay_lidar_cam_th:
            return
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=self.image, desired_encoding='bgr8')
        cloud = pc2.read_points(self.pc_laser)
        cloud_numpy = np.zeros((cloud.shape[0], 4))
        cloud_numpy[:, 0] = cloud['x']
        cloud_numpy[:, 1] = cloud['y']
        cloud_numpy[:, 2] = cloud['z']
        cloud_numpy[:, 3] = cloud['intensity']
        labels = np.unique(cloud_numpy[:, 3])
        for l in labels:
            cluster = cloud_numpy[cloud_numpy[:, 3] == l][:, :3]
            theta = np.arctan2(cluster[:, 1], cluster[:, 0])
            theta_mask = (theta > -np.pi/3) & (theta < np.pi/3)
            cluster = cluster[theta_mask]
            cluster = transform_frame(
                (-LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF[0], LIDAR_WAMV_LINK_TO_FRONT_LEFT_CAMERA_LINK_TF[1].T), cluster)
            print(cluster.shape)
            pixs = XYZ_to_UV(self.K, cluster)
            pixs_mask = (pixs[:, 1] < cv_image.shape[0]) & (
                pixs[:, 0] < cv_image.shape[1]) & (pixs[:, 0] > 0) & (pixs[:, 1] > 0)
            pixs = pixs[pixs_mask]
            cv_image[pixs[:, 1], pixs[:, 0]] = [0, 0, 255]
            # rect = cv2.minAreaRect(pixs)
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)
            # cv_image = cv2.drawContours(cv_image,[box],0,(0,255,255),2)
        cv2.imshow('IMG', cv_image)
        cv2.waitKey(1)

    def predict(self, img_rgb):
        data = self.model.predict(img_rgb)
        for infer in data:
            for xyxy, cls, conf in zip(infer.boxes.xyxy, infer.boxes.cls, infer.boxes.conf):
                conf = round(float(conf.cpu().numpy()), 3)
                # if conf < 0.25:
                #     continue
                pixs = xyxy.cpu().numpy()
                l = int(cls.cpu().numpy())
                p1, p2 = np.int32(pixs[:2]), np.int32(pixs[-2:])
                def segment(x): return (x > p1).all(
                    axis=1) & (x < p2).all(axis=1)


def main(args=None):
    rclpy.init(args=args)
    perception = Perception('')
    rclpy.spin(perception)
    perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
