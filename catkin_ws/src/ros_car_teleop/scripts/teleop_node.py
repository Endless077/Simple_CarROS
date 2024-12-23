#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ros_car_msgs.msg import MoveControls, SpeedControls

###################################################################################################

class TeleopNode:
    """
    Teleoperation node for controlling the robot's movement through commands and set speeds.
 
    A ROS node to handle teleoperation commands for a robot. It processes movement and speed 
    commands and publishes the appropriate Twist messages to control the robot.
    """
    def __init__(self):
        """
        Initializes the TeleopNode.
        """
        rospy.loginfo("Init the TeleopNode.")

        # Subscriber for movement commands
        self.sub_cmd = rospy.Subscriber("/teleop_cmd", MoveControls, self.cmd_callback)
        
        # Subscriber for setting linear/angular speeds
        self.sub_speed = rospy.Subscriber("/set_speed", SpeedControls, self.speed_callback)
        
        # Publisher for cmd_vel (Twist messages)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Set default speed values initially
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

    def cmd_callback(self, msg):
        """
        Callback for processing movement commands. Converts MoveControls messages into Twist
        messages and publishes them to the /cmd_vel topic.
        
        Args:
            msg (MoveControls): The movement command message.
        """
        # Create a Twist message to represent movement commands
        twist = Twist()

        if msg.command == MoveControls.FORWARD:
            # Move forward with the current linear speed
            twist.linear.x = self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.BACKWARD:
            # Move backward with the current linear speed
            twist.linear.x = -self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.LEFT:
            # Turn left with the current angular speed
            twist.linear.x = 0.0
            twist.angular.z = self.current_angular_speed
        elif msg.command == MoveControls.RIGHT:
            # Turn right with the current angular speed
            twist.linear.x = 0.0
            twist.angular.z = -self.current_angular_speed
        elif msg.command == MoveControls.STOP:
            # Stop the robot by setting linear and angular speeds to zero
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message to the /cmd_vel topic
        self.pub.publish(twist)

    def speed_callback(self, msg):
        """
        Callback for processing speed updates. Updates the linear and angular speed values
        based on the SpeedControls message.
        
        Args:
            msg (SpeedControls): The speed control message containing linear and angular speeds.
        """
        # Directly assign the speed values from the message
        self.current_linear_speed = msg.linear_speed
        self.current_angular_speed = msg.angular_speed

        # Log the updated speed values
        rospy.loginfo("Speed changed: linear=%.2f, angular=%.2f" % (self.current_linear_speed, self.current_angular_speed))

###################################################################################################

if __name__ == '__main__':
    """
    Main function to initialize and run the TeleopNode.
    """
    # Initialize the ROS node with the name 'teleop_node'
    rospy.init_node('teleop_node')

    # Create an instance of the TeleopNode class
    node = TeleopNode()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
