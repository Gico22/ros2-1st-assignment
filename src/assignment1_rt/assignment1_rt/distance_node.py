import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math as mt

from time import time

class Distance(Node):

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
        self.x1, self.x2, self.y1, self.y2 = Float32(), Float32(), Float32(), Float32()
        
    def distance_control(self):
        self.d = mt.sqrt((self.x1 - self.x2)**2 + (self.y1 - self.y2)**2)
        self.d_publisher.publish(self.d)

        # check if the turtles are too close from each other
        if self.d <= 1:
            self.v1_publisher.publish(self.stop)
            self.v2_publisher.publish(self.stop)
            self.get_logger().info('Stopping the turtles')
        
        # check if the turtles are too close from the boundaries
        if self.x1 < 1 or self.x1 > 10 or self.y1 < 1 or self.y1 > 10:
            self.v1_publisher.publish(self.stop)
            self.get_logger().info('Stopping turle1')
        
        if self.x2 < 1 or self.x2 > 10 or self.y2 < 1 or self.y2 > 10:
            self.v2_publisher.publish(self.stop)
            self.get_logger().info('Stopping turle2')

    def turtle1_callback(self, msg1):
        self.x1 = msg1.x
        self.y1 = msg1.y
    
    def turtle2_callback(self, msg2):
        self.x2 = msg2.x
        self.y2 = msg2.y
        
def main(args=None):
    rclpy.init(args=args)
    Dist = Distance()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()