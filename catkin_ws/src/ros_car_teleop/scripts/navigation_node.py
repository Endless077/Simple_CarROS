#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ros_car_msgs.msg import MoveControls, SpeedControls

###################################################################################################

class TeleopNode:
    def __init__(self):
        # Read parameters from the parameter server
        self.current_linear_speed = rospy.get_param('/default_linear_speed', 0.5)
        self.current_angular_speed = rospy.get_param('/default_angular_speed', 1.0)

        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        self.last_waypoint = rospy.get_param('/last_waypoint', [-1.0, -1.0])
        self.next_waypoint = rospy.get_param('/next_waypoint', [1.0, 1.0])

        rospy.loginfo("Current Waypoint: %s", str(self.current_waypoint))
        rospy.loginfo("Last Waypoint: %s", str(self.last_waypoint))
        rospy.loginfo("Next Waypoint: %s", str(self.next_waypoint))

        # Subscribers and Publisher for controlling the robot
        self.sub_cmd = rospy.Subscriber("/teleop_cmd", MoveControls, self.cmd_callback)
        self.sub_speed = rospy.Subscriber("/set_speed", SpeedControls, self.speed_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def speed_callback(self, msg):
        # Update linear and angular speeds from received message
        self.current_linear_speed = msg.linear_speed
        self.current_angular_speed = msg.angular_speed

        rospy.loginfo("Speed changed: linear=%.2f, angular=%.2f" % (self.current_linear_speed, self.current_angular_speed))

    def cmd_callback(self, msg):
        # Create a Twist message to represent movement commands
        twist = Twist()
        
        if msg.command == MoveControls.FORWARD:
            # Move forward
            twist.linear.x = self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.BACKWARD:
            # Move backward
            twist.linear.x = -self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.LEFT:
            # Turn left
            twist.linear.x = 0.0
            twist.angular.z = self.current_angular_speed
        elif msg.command == MoveControls.RIGHT:
            # Turn right
            twist.linear.x = 0.0
            twist.angular.z = -self.current_angular_speed
        elif msg.command == MoveControls.STOP:
            # Stop movement
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message to the /cmd_vel topic
        self.pub.publish(twist)

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'teleop_node'
    rospy.init_node('teleop_node')

    # Create an instance of the TeleopNode class
    node = TeleopNode()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
