# Use this if u wanna run multiple nodes in same executable
import rclpy

from testpub import Thrust_Publisher
from testsub import GpsSubscriber

from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    try:
        gps_subscriber = GpsSubscriber()
        thrust_publisher = Thrust_Publisher()
        

        executor = SingleThreadedExecutor()
        executor.add_node(gps_subscriber)
        executor.add_node(thrust_publisher)
       

        try:
            
            executor.spin()
            
        finally:
            executor.shutdown()
            thrust_publisher.destroy_node()
            gps_subscriber.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    