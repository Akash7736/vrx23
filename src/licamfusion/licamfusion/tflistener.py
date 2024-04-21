
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'turtle1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        # self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        # self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        # self.turtle_spawned = False

        # Create turtle2 velocity publisher
        # self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'wamv/lidar_wamv_link'
        to_frame_rel = 'wamv/left_camera_link_optical'

    
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            print([t.transform.translation.y,t.transform.translation.x])
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        
        

   


def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    try:
        rclpy.spin(node)
        print("ttt")
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()