import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class UI(Node):
    def __init__(self):
        super().__init__('UI')

        # create publishers for the two turtles
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.t_publisher = self.create_publisher(Int32, 'moving_turtle', 10)

        # initializing variables
        self.turtle_num = None      # moving turtle
        self.twist = Twist()        # turtle velocity (linear and angular)
        self.stop = Twist()         # turtle stop command (twist with '0' values)

        # create timer
        self.timer = self.create_timer(0.1, self.user_interface) 
        
    def user_interface(self):

        # user input for selecting the turtle
        while True:
            turtle = input("Select a turtle, available options '1' or '2': ")
            if turtle in {"1", "2"}:
                self.turtle_num = int(turtle)
                break
            print("Invalid input, please select '1' or '2'")

        # user input for selecting velocity and angular velocity
        self.twist.linear.x = float(input("Select the linear velocity: "))
        self.twist.angular.z = float(input("Select the angular velocity: "))

        # publish the moving turtle
        msg_t = Int32()
        msg_t.data = self.turtle_num
        self.t_publisher.publish(msg_t)

        # publish messages for 1 second
        if self.turtle_num == 1:
            self.publisher1.publish(self.twist)
            time.sleep(1.0)
        else:
            self.publisher2.publish(self.twist)
            time.sleep(1.0)
                
        # stop the turtle after 1 second
        if self.turtle_num == 1:
            self.publisher1.publish(self.stop)
        else:
            self.publisher2.publish(self.stop)
        
        # publish 'no moving turtle' as 0
        msg_t.data = self.turtle_num = 0
        self.t_publisher.publish(msg_t)
        self.get_logger().info("Movement finished")

        # reinitialize twist to remove previous values
        self.twist = Twist()

def main(args = None):
    rclpy.init(args = args)
    Ui = UI()
    rclpy.spin(Ui)
    rclpy.shutdown()

if __name__ == '__main__':
    main()