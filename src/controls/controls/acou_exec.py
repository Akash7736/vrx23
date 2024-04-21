# Use this if u wanna run multiple nodes in same executable
import rclpy

from acoskp import PDController
from acoustics import Acoustics

from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    try:
        aco = Acoustics()
        pdc  = PDController()
        
        

        executor = SingleThreadedExecutor()
        executor.add_node(aco)
        executor.add_node(pdc)
        
       

        try:
            
            executor.spin()
            
        finally:
            executor.shutdown()
            aco.destroy_node()
            pdc.destroy_node()
            

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    