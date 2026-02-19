"""
A node that controls robot to move in a pattern of square.
"""

import math
import rclpy
from rclpy.node import Node

# We will use Twist message which has two vectors that consists of x, y, z values named linear and anguler respectively
from geometry_msgs.msg import Twist


# Define the node
class SquareDriverNode(Node):
    """
    A node for square driving.
    """

    def __init__(self):
        super().__init__('square_driver_node')

        # Create a publisher to publish message
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create timer object
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        
        # Create other variables
        self.counter = 0
        self.state = "FORWARD"
        self.distance = 1                # 1 m
        self.linear_speed = 0.2          # 0.2 m/s linear speed    (x axis)
        self.angular_speed = 0.5         # 0.5 rad/s angular speed (z axis)
        self.forward_duration = self.distance / self.linear_speed
        self.rotation_duration = (math.pi / 2) / self.angular_speed
        self.state_start_time = self.get_clock().now()


    # Define a timer callback to actually make things happen
    def timer_callback(self):
        """
        Timer callback function for SquareDriverNode
        """

        # Define message and a time passed variable to manage time
        msg = Twist()
        time_passed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9     # Gives us seconds

        # Manage states
        if self.state == "FORWARD":
            # Assign linear speed
            msg.linear.x = self.linear_speed
            
            # Change state and reset timer
            if time_passed >= self.forward_duration:
                self.state = "STOP_AFTER_FORWARD"
                self.state_start_time = self.get_clock().now()
        
        elif self.state == "STOP_AFTER_FORWARD":
            # Reset linear speed
            msg.linear.x = 0.0

            # Change state and reset timer
            if time_passed >= 0.5:
                self.state = "TURN"
                self.state_start_time = self.get_clock().now()
        
        elif self.state == "TURN":
            # Assign angular speed
            msg.angular.z = self.angular_speed

            # Change state and reset timer
            if time_passed >= self.rotation_duration:
                self.state = "STOP_AFTER_TURN"
                self.state_start_time = self.get_clock().now()

        elif self.state == "STOP_AFTER_TURN":
            # Reset angular speed
            msg.angular.z = 0.0

            # Change state and reset timer
            if time_passed >= 0.5:
                # Increment the counter
                self.counter += 1

                # Check if the counter is greater than 4
                if self.counter >= 4:
                    self.state = "DONE"
                else:
                    self.state = "FORWARD"
                    self.state_start_time = self.get_clock().now()

        elif self.state == "DONE":
            #Cancel the timer
            self.timer.cancel()

        # Print logs to terminal
        self.get_logger().info(f"State: {self.state} -- Linear speed: {msg.linear.x, msg.linear.y, msg.linear.z} -- Angular speed: {msg.angular.x, msg.angular.y, msg.angular.z}")

        # Publish the message
        self.pub.publish(msg)


# Define the main function to run the node
def main():
    # Implement a node lifecycle
    rclpy.init()
    node = SquareDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Run the node
if __name__ == '__main__':
    main()