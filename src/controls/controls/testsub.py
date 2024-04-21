import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix


class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10)
        self.subscription  # prevent unused variable warning

    def gps_callback(self, msg):
        print("Hi")
        print(msg)
        # self.get_logger().info(float(msg.latitude))
        # self.get_logger().debug(msg.latitude)


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GpsSubscriber()

    rclpy.spin(gps_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()