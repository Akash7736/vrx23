import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import time

class thrust_Publisher(Node):

    def __init__(self):
        super().__init__('thrust_publisher')
        self.publisher_ = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     

    def timer_callback(self):
        msg = Float64()
        msg.data = float(0) 
        self.publisher_.publish(msg)
        # self.get_logger().info(msg)
      


def main(args=None):
    rclpy.init(args=args)

    thrust_publisher = thrust_Publisher()

    rclpy.spin(thrust_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrust_publisher.destroy_node()
    rclpy.shutdown()

class PDController:

    def __init__(self,kp,kd):
        self.kp = kp 
        self.kd = kd 
        self.lasterror = 0
        self.last_time = 0
        self.TA_matrix = np.array([[0.707, -0.707, -0.707, 0.707],
                                    [0.707, 0.707, 0.707, 0.707],
                                    [0.35,  0.35,  0.35,  0.35]])
                                   
        self.pseudo_TA_inv = np.linalg.pinv(self.TA_matrix)
        self.parama1 = np.array([ 1.52503318e-05, -4.91422601e-02,  3.93300223e+01])
        self.parama2 = np.array([ 1.61019834e-05, -4.23940889e-02,  2.71121120e+01])
        self.paramb1 = np.array([ 1.67759057e-05, -5.46903629e-02,  4.41751206e+01])
        self.paramb2 = np.array([ 2.12621123e-05, -5.82516683e-02,  3.93836637e+01])
        


    def update(self,error):
        present_time = time.time()
        dt = present_time - self.last_time # change in time
        dt = 0.1
        derivative = (error- self.lasterror)/dt # rate of change of error
        self.lasterror = error
        self.last_time = present_time

        return (self.kp * error) + (self.kd * derivative)
    

if __name__ == '__main__':
    main()