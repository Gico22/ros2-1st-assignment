'''import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from time import time

class UI(Node):
    def __init__(self):
        super().__init__('UI')

        # create publishers for the two turtles
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.t_publisher = self.create_publisher(Int32, 'moving_turtle', 10)

        # create subscriber
        self.d_subscription = self.create_subscription(Float32, 'Distance', self.d_callback, 10)

        # initializing variables
        self.turtle_num = None      # turtle number
        self.twist = Twist()        # turtle velocity (linear and angular)
        self.stop = Twist()         # turtle stop command (twist with '0' values)
        self.start = 0.0            # time
        self.distance = None        # distance

        # initializing flags
        self.timer = None
        self.tim_active = False         
        self.command_received = False
        
    def user_interface(self):

        # user input for selecting the turtle
        while True:
            turtle = input("Select a turtle, available options: '1' or '2'")
            if turtle in {"1", "2"}:
                self.turtle_num = int(turtle)
                break
            print("Invalid input, please select '1' or '2'")

        # user input for selecting velocity and angular velocity
        self.twist.linear.x = float(input("Select the linear velocity"))
        self.twist.angular.z = float(input("Select the angular velocity"))
        self.command_received = True

        # publish the moving turtle
        msg_t = Int32()
        msg_t.data = self.turtle_num
        self.t_publisher.publish(msg_t)

        # publish the velocity
        self.start = time()
        self.tim_active = True
        self.timer = self.create_timer(0.1, self.vel_pub) 
        
    def vel_pub(self):

        # publish messages for 1 second
        if time() - self.start < 1:
            if self.turtle_num == 1:
                self.publisher1.publish(self.twist)
            else:
                self.publisher2.publish(self.twist)
        else:   
            # stop the turtle after 1 second
            if self.turtle_num == 1:
                self.publisher1.publish(self.stop)
            else:
                self.publisher2.publish(self.stop)
            
            # publish 'no moving turtle' as 0
            msg = Int32()
            msg.data = 0
            self.t_publisher.publish(msg)

            # stop timer
            self.timer.cancel()
            self.tim_active = False
            self.turtle_num = 0
            self.timer = None
            self.get_logger().info("Movement finished")

            # reinitialize twist to remove previous values
            self.twist = Twist()

    def d_callback(self, dist):
        self.distance = dist.data
        if self.distance <= 0.5 and self.tim_active:
            self.twist.linear.x = -self.twist.linear.x
            self.twist.angular.x = -self.twist.angular.x


def main(args = None):
    rclpy.init(args = args)
    Ui = UI()

    while rclpy.ok():
        Ui.user_interface()
        while Ui.tim_active and rclpy.ok():
            rclpy.spin_once(Ui)
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class UI(Node):
    def __init__(self):
        super().__init__('ui')

        self.pub_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pub_moving = self.create_publisher(Int32, 'moving_turtle', 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.active = False
        self.active_turtle = None
        self.cmd = Twist()
        self.stop_cmd = Twist()
        self.stop_cmd.linear.x = 0.0
        self.stop_cmd.angular.z = 0.0
        self.end_time = 0.0

    def loop(self):
        # If currently sending a command
        if self.active:
            now = time.time()

            if now < self.end_time:
                if self.active_turtle == 1:
                    self.pub_t1.publish(self.cmd)
                else:
                    self.pub_t2.publish(self.cmd)
            else:
                # Stop the turtle
                if self.active_turtle == 1:
                    self.pub_t1.publish(self.stop_cmd)
                else:
                    self.pub_t2.publish(self.stop_cmd)

                self.active = False
                self.get_logger().info("Command finished. Enter new command.")
            return

        # Otherwise wait for user input (non-blocking)
        turtle = input("Choose turtle (1 or 2): ")
        if turtle not in ['1', '2']:
            print("Invalid, try again.")
            return

        self.active_turtle = int(turtle)
        lin = float(input("Linear velocity: "))
        ang = float(input("Angular velocity: "))

        self.cmd.linear.x = lin
        self.cmd.angular.z = ang

        # publish which turtle is moving
        msg = Int32()
        msg.data = self.active_turtle
        self.pub_moving.publish(msg)

        # activate 1-second command
        self.end_time = time.time() + 1.0
        self.active = True

def main():
    rclpy.init()
    node = UI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
