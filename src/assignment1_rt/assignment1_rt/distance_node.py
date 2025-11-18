import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import time

class distance(Node):

    def __init__(self):
        super().__init__('Distance')
        self.d = Float32()
        self.stop = Twist()
        self.v_publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.v_publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10) 
        self.d_publisher = self.create_publisher(Float32, 'distance', 10) 
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.dist)


        
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()