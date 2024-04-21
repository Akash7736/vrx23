import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import numpy as np
# mesh = o3d.geometry.TriangleMesh.create_sphere()
# mesh.compute_vertex_normals()
# # o3d.visualization.draw_geometries([mesh])
# pcd = mesh.sample_points_uniformly(number_of_points=500)
# o3d.visualization.draw_geometries([pcd])


class Fuse(Node):

    def __init__(self):
        super().__init__('od')
        self.subscriptionvoxel = self.create_subscription(PointCloud2,'/myvoxel',self.voxel_callback,10)
        self.subscriptionvoxel
   
    def voxel_callback(self, msg:PointCloud2):


        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(
            (msg))
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                                 ransac_n=3,
                                                 num_iterations=100)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        output = np.array(outlier_cloud.points)

        # with o3d.utility.VerbosityContextManager(
        #         o3d.utility.VerbosityLevel.Debug) as cm:
        #     labels = np.array(
        #         pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        # max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
        print(output)
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([pcd],
        #                                 zoom=0.455,
        #                                 front=[-0.4999, -0.1659, -0.8499],
        #                                 lookat=[2.1813, 2.0619, 2.0999],
        #                                 up=[0.1204, -0.9852, 0.1215])



def main(args=None):
    rclpy.init(args=args)

    fl = Fuse()


    rclpy.spin(fl)

    # skp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()