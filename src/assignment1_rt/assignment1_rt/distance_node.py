import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from time import time

class distance(Node):

    def __init__(self):
        super().__init__('Distance')
        # create publishers for the two turtles and distance
        self.v1_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.v2_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10) 
        self.d_publisher = self.create_publisher(Float32, 'distance', 10) 

        # create subscribers
        self.p1_subscription = self.create_subscription(Pose, '/turtle1/Pose', self.turtle1_callback, 10)
        self.p2_subscription = self.create_subscription(Pose, '/turtle2/Pose', self.turtle2_callback, 10)
        
        # create timer
        self.timer = self.create_timer(0.05, self.distance_control)

        # initializing variables
        self.d = Float32()          # distance
        self.stop = Twist()         # turtle stop command (twist with '0' values)
        x1, x2, y1, y2 = Float32(), Float32(), Float32(), Float32()
        


    def turtle1_callback(self, msg1):
        x1 = msg1.x
        y1 = msg1.y
    
    def turtle2_callback(self, msg2):
        x2 = msg2.x
        y2 = msg2.y
        
        
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()