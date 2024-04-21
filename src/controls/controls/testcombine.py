import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64



class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        # self.pub_callback
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(NavSatFix,'/wamv/sensors/gps/gps/fix',self.gps_callback,10)
        self.subscription  # prevent unused variable warning
     
    def timer_callback(self):
        msg = Float64()
        msg.data = float(0) 
        print("Hello")
        self.publisher_.publish(msg)
        # self.get_logger().info(msg)

    def pub_callback(self):
        msg = Float64
        msg.data = float(0)
        print("Hello")
        self.publisher_.publish(msg)

    def gps_callback(self, msg):
        print("Hi")
        print(msg)
        # self.get_logger().info(float(msg.latitude))
        # self.get_logger().debug(msg.latitude)

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    # controller.pub_callback


    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   

    # rclpy.spin(thrust_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()