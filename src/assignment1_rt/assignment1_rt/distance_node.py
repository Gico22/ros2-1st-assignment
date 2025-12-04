import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import math

class Distance(Node):
    def __init__(self):
        super().__init__('Distance')
        # create publishers for the two turtles and distance
        self.v1_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.v2_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10) 
        self.d_publisher = self.create_publisher(Float32, 'distance', 10) 

        # create subscribers
        self.p1_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.p2_subscription = self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)
        self.t_subscription = self.create_subscription(Int32, 'moving_turtle', self.turtle_num_callback, 10)
        self.vel1_subscription = self.create_subscription(Twist, '/turtle1/cmd_vel', self.vel1_callback, 10)
        self.vel2_subscription = self.create_subscription(Twist, '/turtle2/cmd_vel', self.vel2_callback, 10)

        # initializing variables
        self.distance = 0.0                             # distance
        self.safety_twist = Twist()                     # input velocity (linear and angular)
        self.stop = Twist()                             # turtle stop command (twist with '0' values)
        self.turtle_num = 0                             # moving turtle
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0     # turtles positions
        self.not_safe = False

        # initializing flags
        self.pose1_received = False
        self.pose2_received = False

        # create timer
        self.timer = self.create_timer(0.01, self.distance_control)

    def distance_control(self):
        if not (self.pose1_received and self.pose2_received):
            return

        self.distance = math.sqrt((self.x1 - self.x2)**2 + (self.y1 - self.y2)**2)
        msg_d = Float32()
        msg_d.data = self.distance
        self.d_publisher.publish(msg_d)

        # check if the turtles are too close from each other
        if self.distance <= 0.5:
            self.not_safe = True
            if self.turtle_num == 1:
                self.v1_publisher.publish(self.safety_twist)
                self.get_logger().info('Stopping turtle1')
            elif self.turtle_num == 2:
                self.v2_publisher.publish(self.safety_twist)
                self.get_logger().info('Stopping turtle2')

        # check if the turtles are too close from the boundaries
        if (self.x1 < 1 or self.x1 > 10 or self.y1 < 1 or self.y1 > 10):
            self.not_safe = True
            self.v1_publisher.publish(self.safety_twist)
            self.get_logger().info('Stopping turtle1')

        if (self.x2 < 1 or self.x2 > 10 or self.y2 < 1 or self.y2 > 10):
            self.not_safe = True
            self.v2_publisher.publish(self.safety_twist)
            self.get_logger().info('Stopping turtle2')

        if self.safe_pose() and self.not_safe:
            if self.turtle_num == 1:
                self.v1_publisher.publish(self.stop)
                self.get_logger().info('Stopping turtle1')
            elif self.turtle_num == 2:
                self.v2_publisher.publish(self.stop)
                self.get_logger().info('Stopping turtle2')
            self.not_safe = False

    def pose1_callback(self, msg1):
        self.x1 = msg1.x
        self.y1 = msg1.y
        self.pose1_received = True

    def pose2_callback(self, msg2):
        self.x2 = msg2.x
        self.y2 = msg2.y
        self.pose2_received = True

    def turtle_num_callback(self, msg_t):
        self.turtle_num = msg_t.data

    def vel1_callback(self, vel1):
        self.safety_twist.linear.x = -vel1.linear.x
        self.safety_twist.angular.z = -vel1.angular.z

    def vel2_callback(self, vel2):
        self.safety_twist.linear.x = -vel2.linear.x
        self.safety_twist.angular.z = -vel2.angular.z

    def safe_pose(self):
        return self.distance > 1.2 and 1.2 < self.x1 < 9.8 and 1.2 < self.x2 < 9.8 and 1.2 < self.y1 < 9.8 and 1.2 < self.y2 < 9.8

def main(args=None):
    rclpy.init(args = args)
    Dist = Distance()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()