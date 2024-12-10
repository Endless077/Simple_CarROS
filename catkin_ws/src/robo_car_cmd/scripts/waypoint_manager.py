#!/usr/bin/env python3
import math
import rospy
import random
from nav_msgs.msg import Odometry

###################################################################################################

class WaypointManager:
    def __init__(self):
        # Read initial waypoints from the parameter server
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        self.last_waypoint = rospy.get_param('/last_waypoint', [-1.0, -1.0])
        self.next_waypoint = rospy.get_param('/next_waypoint', [2.0, 1.5])

        # Distance threshold to determine if the robot has reached the current waypoint
        self.arrival_threshold = 0.2

        # Subscriber to the /odom topic to monitor the robot's position
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Waypoint Manager started.")
        rospy.loginfo("Current waypoint: %s", str(self.current_waypoint))
        rospy.loginfo("Next waypoint: %s", str(self.next_waypoint))

    def odom_callback(self, msg):
        # Extract the robot's current position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate the distance to the current waypoint
        dx = x - self.current_waypoint[0]
        dy = y - self.current_waypoint[1]
        dist = math.sqrt(dx*dx + dy*dy)

        # If the robot is within the arrival threshold, update the waypoints
        if dist < self.arrival_threshold:
            rospy.loginfo("Reached current waypoint: %s", str(self.current_waypoint))
            
            # Update the waypoints
            self.last_waypoint = self.current_waypoint
            self.current_waypoint = self.next_waypoint
            
            # Generate a new random waypoint near the current waypoint
            # New waypoint is within +/- 2 meters range
            rand_x = self.current_waypoint[0] + random.uniform(-2, 2)
            rand_y = self.current_waypoint[1] + random.uniform(-2, 2)
            self.next_waypoint = [rand_x, rand_y]

            rospy.loginfo("New current waypoint: %s", str(self.current_waypoint))
            rospy.loginfo("New next waypoint (random): %s", str(self.next_waypoint))

            # Update the parameters on the parameter server
            rospy.set_param('/last_waypoint', self.last_waypoint)
            rospy.set_param('/current_waypoint', self.current_waypoint)
            rospy.set_param('/next_waypoint', self.next_waypoint)
            rospy.set_param('/next_waypoint', self.next_waypoint)

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'waypoint_manager'
    rospy.init_node('waypoint_manager')

    # Create an instance of the WaypointManager class
    manager = WaypointManager()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
