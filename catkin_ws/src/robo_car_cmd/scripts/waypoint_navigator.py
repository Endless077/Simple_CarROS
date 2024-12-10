#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

###################################################################################################

class WaypointNavigator:
    def __init__(self):
        # Parameters
        self.arrival_threshold = 0.2  # Distance threshold to consider the waypoint reached
        self.linear_speed = 0.3       # Base linear speed
        self.angular_speed = 0.5      # Base angular speed
        
        # Read the current waypoint from the parameter server
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])

        # Robot's state (position and orientation)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Publisher for /cmd_vel topic to control the robot's velocity
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to /odom topic to receive the robot's odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Timer for periodic control loop execution (every 0.1 seconds)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Waypoint Navigator started.")

    def odom_callback(self, msg):
        # Extract the robot's current position and orientation from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        
        # Convert quaternion orientation to roll, pitch, yaw
        (roll, pitch, yaw) = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Store yaw (rotation around the z-axis)
        self.robot_yaw = yaw

    def control_loop(self, event):
        # Update the current waypoint from the parameter server (to allow dynamic changes)
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        
        # Calculate the distance and angle to the current waypoint
        dx = self.current_waypoint[0] - self.robot_x
        dy = self.current_waypoint[1] - self.robot_y
        dist = math.sqrt(dx*dx + dy*dy)  # Euclidean distance to the waypoint

        # If the robot is close enough to the waypoint, stop the robot
        if dist < self.arrival_threshold:
            self.stop_robot()
            return

        # Calculate the desired angle to the waypoint
        theta_des = math.atan2(dy, dx)
        error_yaw = theta_des - self.robot_yaw

        # Normalize the yaw error to the range [-pi, pi]
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        # Create a Twist message to control the robot
        cmd = Twist()
        # If the yaw error is significant, rotate the robot in place
        if abs(error_yaw) > 0.1:
            # Adjust direction
            cmd.angular.z = self.angular_speed * (error_yaw / abs(error_yaw))
            cmd.linear.x = 0.0  # No forward motion while turning
        else:
            # The robot is well-oriented; move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        # Publish the velocity command to the /cmd_vel topic
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        # Publish a Twist message with zero velocity to stop the robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'waypoint_navigator'
    rospy.init_node('waypoint_navigator')

    # Create an instance of the WaypointNavigator class
    navigator = WaypointNavigator()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
