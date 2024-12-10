#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from robo_car_msgs.msg import MoveControl, SpeedControl

###################################################################################################

class TeleopNode:
    def __init__(self):
        # Init the TeleopNode
        rospy.loginfo("Init the TeleopNode.")

        # Subscriber for movement commands
        self.sub_cmd = rospy.Subscriber("/teleop_cmd", MoveControl, self.cmd_callback)
        # Subscriber for setting linear/angular speeds
        self.sub_speed = rospy.Subscriber("/set_speed", SpeedControl, self.speed_callback)
        
        # Publisher for cmd_vel (Twist messages)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Set default speed values initially
        self.current_linear_speed = 0.5
        self.current_angular_speed = 1.0

    def cmd_callback(self, msg):
        # Create a Twist message to represent movement commands
        twist = Twist()

        if msg.command == MoveControl.FORWARD:
            # Move forward with the current linear speed
            twist.linear.x = self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControl.BACKWARD:
            # Move backward with the current linear speed
            twist.linear.x = -self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControl.LEFT:
            # Turn left with the current angular speed
            twist.linear.x = 0.0
            twist.angular.z = self.current_angular_speed
        elif msg.command == MoveControl.RIGHT:
            # Turn right with the current angular speed
            twist.linear.x = 0.0
            twist.angular.z = -self.current_angular_speed
        elif msg.command == MoveControl.STOP:
            # Stop the robot by setting linear and angular speeds to zero
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message to the /cmd_vel topic
        self.pub.publish(twist)

    def speed_callback(self, msg):
        # Directly assign the speed values from the message
        self.current_linear_speed = msg.linear_speed
        self.current_angular_speed = msg.angular_speed

        # Log the updated speed values
        rospy.loginfo("Speed changed: linear=%.2f, angular=%.2f" % (self.current_linear_speed, self.current_angular_speed))

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'teleop_node'
    rospy.init_node('teleop_node')

    # Create an instance of the TeleopNode class
    node = TeleopNode()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
