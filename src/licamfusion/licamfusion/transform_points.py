#!/usr/bin/env python3


# import modules
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped


class TF2Example(Node):
    def __init__(self):
        super().__init__('tf2_ros_example')

        # define source and target frame


    def transform_point(self, transformation, point_wrt_source):
        point_wrt_target = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_source), transformation).point
        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

    def get_transformation(self, source_frame, target_frame, tf_cache_duration=2.0):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        # get the tf at the first available time
        try:
            transformation = tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            print(transformation)
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('Unable to find the transformation from %s to %s' % (source_frame, target_frame))


def main(args=None):
    rclpy.init(args=args)
    tf2_example = TF2Example()
    source_frame = 'wamv/wamv/base_link'
    target_frame = 'wamv/wamv/lidar_wamv_link'

    # define a source point
    point_wrt_source = Point()
    point_wrt_source.x = 0.1
    point_wrt_source.y = 1.2
    point_wrt_source.z = 2.3

    transformation = tf2_example.get_transformation(source_frame, target_frame)
    # if transformation.transform is not None :
    print(transformation.transform.rotation)
    point_wrt_target = tf2_example.transform_point(transformation, point_wrt_source)
    tf2_example.get_logger().info(str(point_wrt_target))
    print('y')
    rclpy.spin(tf2_example)
    tf2_example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
