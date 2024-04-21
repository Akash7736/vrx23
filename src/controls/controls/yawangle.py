import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import ParamVec
from tf_transformations import euler_from_quaternion

class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.imusubscription = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10)
        self.imusubscription  # prevent unused variable warning
        self.pingersubscription = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.ping_callback,
            10)
        self.pingersubscription  # prevent unused variable warning
        self.yaw = 0
        self.bearing = 0
        self.elevation = 0
        self.range = 0


    def imu_callback(self,data: Imu):
        quaternion = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
        euler = euler_from_quaternion(quaternion) # quaterion to euler angle conversion
        self.yaw = euler[2] # yaw
        # print((self.yaw*180)/np.pi)

    def ping_callback(self,data: ParamVec):
        # data.params[]
        print(data.header.frame_id)
        # # print(dir(data.params[0]))
 
  
        print(di)
        self.bearing = di['bearing']
        self.range = di['range']
        self.elevation = di['elevation']
        # print(di)
        # print(data.params[2].value.double_value) # 0-elevation, 1-bearing, 2-range
        # print(type(data.params[0]))
        # print(f"range:{data.params[2].value.double_value}, bearing:{data.params[1].value.double_value*180/np.pi} elevation:{data.params[0].value.double_value*180/np.pi} ")
        

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = GpsSubscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()