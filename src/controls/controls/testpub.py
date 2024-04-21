import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node
from mymessages.msg import ActionMesg
class Thrust_Publisher(Node):

    def __init__(self):
        super().__init__('thrust_publisher')
        self.publisher_ = self.create_publisher(ActionMesg, '/newtoopic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     

    def timer_callback(self):
        msg = ActionMesg()
        msg.data = [1.0,2.0]
        msg.header.stamp = self.get_clock().now().to_msg()
        print("Hello")
        # print("Hello")
        self.publisher_.publish(msg)
        # self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)

    thrust_publisher = Thrust_Publisher()

    rclpy.spin(thrust_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrust_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()