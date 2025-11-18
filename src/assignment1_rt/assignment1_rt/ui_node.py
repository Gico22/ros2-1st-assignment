import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import time

class UI(Node):
    
    def __init__(self):
        super().__init__('UI')

        # create publishers for the two turtles
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # initializing needed parameters
        self.turtle_num = None      # turtle number
        self.twist = Twist()        # turtle velocity (linear and angular)
        self.stop = Twist()         # turtle stop command (twist with '0' values)
        self.start = 0.0            # time

        # initializing flags
        self.timer = None
        self.tim_active = False         

    def user_interface(self):

        # user input for selecting the turtle
        while True:
            turtle = int(input("Select a turtle, available options: '1' or '2'"))
            if turtle in {"1", "2"}:
                self.turtle_num = int(turtle)
                break
            print("Invalid input, please select '1' or '2'")

        # user input forf selecting velocity
        self.twist.linear.x = float(input("Select the linear velocity"))
        self.twist.angular.z = float(input("Select the angular velocity"))

        # publish on the topic
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
            
            # stop timer
            self.timer.cancel()
            self.tim_active = False
            self.get_logger().info("Movement finished")

def main(args = None):
    rclpy.init(args = args)
    Ui = UI()

    while rclpy.ok():
        Ui.user_interface()
        while Ui.tim_active and rclpy.ok():
            rclpy.spin_once(Ui)
    rclpy.shutdown()

if __name__ == '__main__':
    main()