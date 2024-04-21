import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/mytopic/goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [2.0,5.0]
        # msg.data[1] = 5  
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()