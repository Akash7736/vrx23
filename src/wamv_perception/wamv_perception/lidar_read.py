#/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 #used as the subscription message type
from sensor_msgs_py import point_cloud2 #used to read points
import pclpy as pcl

class LidarRead(Node):

    def __init__(self):
        super().__init__("lidar_read")
        self.subscription = self.create_subscription(PointCloud2,'/wamv/sensors/lidars/lidar_wamv_sensor/points',self.lidar_callback,10)

    def lidar_callback(self,msg:PointCloud2):
        xyz = list(point_cloud2.read_points(msg, field_names=('x','y','z')))
        # Create a VoxelGrid filter
        vox = pcl.filters.VoxelGrid[xyz]()
        vox.setInputCloud(xyz)
        # Set the voxel size (adjust as needed)
        voxel_size = 0.1  # Example voxel size in meters
        vox.setLeafSize(voxel_size, voxel_size, voxel_size)
        
        # Apply the voxel filter
        filtered_xyz = pcl.PointCloud()
        vox.filter(filtered_xyz)
        
        # Now, filtered_xyz contains the downsampled point cloud data
        # You can use filtered_xyz for further processing or analysis

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarRead()
    rclpy.spin(lidar_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()