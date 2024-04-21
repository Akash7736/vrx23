# controls_groupB/src/pose_publisher_plugin.py

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class PosePublisherPlugin(Node):

    def __init__(self):
        super().__init__('pose_publisher_node')
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_topic', 10)
        self.model_name = 'your_model_name'  # Replace with your model's name
        self.model_index = -1  # Initialize the model index

    def model_states_callback(self, msg):
        try:
            # Find the index of the specified model in the ModelStates message
            self.model_index = msg.name.index(self.model_name)
        except ValueError:
            # Model not found in the message
            pass

        if self.model_index >= 0:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'your_frame_id'  # Replace with your frame ID
            pose_msg.pose = msg.pose[self.model_index]
            self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherPlugin()

    # Subscribe to the ModelStates topic to get pose information
    model_states_subscription = node.create_subscription(
        ModelStates, 'gazebo/model_states', node.model_states_callback, 10)

    # Spin the node
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
