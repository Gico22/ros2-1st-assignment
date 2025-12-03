'''import rclpy
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
        self.p1_subscription = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.p2_subscription = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_callback, 10)
        self.t_subscription = self.create_subscription(Int32, 'moving_turtle', self.turtle_num_callback, 10)

        # initializing variables
        self.distance = 0.0                # distance
        self.stop = Twist()         # turtle stop command (twist with '0' values)
        self.moving_t = 0           # moving turtle
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0     # turtles positions

        # initializing flags
        self.pose1_received = False
        self.pose2_received = False
        self.stop_once = False

        # create timer
        self.timer = self.create_timer(0.05, self.distance_control)

    def distance_control(self):
        if not (self.pose1_received and self.pose2_received):
            return

        self.distance = math.sqrt((self.x1 - self.x2)**2 + (self.y1 - self.y2)**2)
        msg_d = Float32()
        msg_d.data = self.distance
        self.d_publisher.publish(msg_d)

        # check if the turtles are too close from each other
        if self.distance <= 0.5 and not self.stop_once:
            if self.moving_t == 1:
                self.v1_publisher.publish(self.stop)
                self.get_logger().info('Stopping turtle1')
            elif self.moving_t == 2:
                self.v2_publisher.publish(self.stop)
                self.get_logger().info('Stopping turtle2')
            self.stop_once = True

        # check if the turtles are too close from the boundaries
        if self.x1 < 1 or self.x1 > 10 or self.y1 < 1 or self.y1 > 10:
            self.v1_publisher.publish(self.stop)
            self.get_logger().info('Stopping turtle1')

        if self.x2 < 1 or self.x2 > 10 or self.y2 < 1 or self.y2 > 10:
            self.v2_publisher.publish(self.stop)
            self.get_logger().info('Stopping turtle2')

    def turtle1_callback(self, msg1):
        self.x1 = msg1.x
        self.y1 = msg1.y
        self.pose1_received = True

    def turtle2_callback(self, msg2):
        self.x2 = msg2.x
        self.y2 = msg2.y
        self.pose2_received = True

    def turtle_num_callback(self, msg_t):
        self.moving_t = msg_t.data
        self.stop_once = False

def main(args=None):
    rclpy.init(args=args)
    Dist = Distance()
    rclpy.spin(Dist)
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
import math

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        self.sub_t1 = self.create_subscription(Pose, '/turtle1/pose', self.cb_t1, 10)
        self.sub_t2 = self.create_subscription(Pose, '/turtle2/pose', self.cb_t2, 10)
        self.sub_moving = self.create_subscription(Int32, 'moving_turtle', self.cb_moving, 10)

        self.pub_dist = self.create_publisher(Float32, 'distance', 10)

        self.pub_t1_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_t2_cmd = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.t1 = None
        self.t2 = None
        self.moving = None

        self.stop_cmd = Twist()

        self.timer = self.create_timer(0.1, self.check_distance)

    def cb_t1(self, msg):
        self.t1 = msg

    def cb_t2(self, msg):
        self.t2 = msg

    def cb_moving(self, msg):
        self.moving = msg.data

    def stop_moving_turtle(self):
        if self.moving == 1:
            self.pub_t1_cmd.publish(self.stop_cmd)
        elif self.moving == 2:
            self.pub_t2_cmd.publish(self.stop_cmd)

    def check_distance(self):
        if self.t1 is None or self.t2 is None:
            return

        # compute distance
        d = math.sqrt((self.t1.x - self.t2.x)**2 + (self.t1.y - self.t2.y)**2)

        # publish distance
        msg = Float32()
        msg.data = d
        self.pub_dist.publish(msg)

        # thresholds
        if d < 1.0:
            self.get_logger().warn("Too close! Stopping moving turtle.")
            self.stop_moving_turtle()

        # boundary check
        for t in [self.t1, self.t2]:
            if t.x < 1.0 or t.x > 10.0 or t.y < 1.0 or t.y > 10.0:
                self.get_logger().warn("Boundary reached! Stopping moving turtle.")
                self.stop_moving_turtle()
                break

def main():
    rclpy.init()
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
