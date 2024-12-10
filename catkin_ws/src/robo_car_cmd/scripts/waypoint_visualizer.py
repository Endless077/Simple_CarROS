#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

###################################################################################################

class WaypointVisualizer:
    def __init__(self):
        # Publisher to the /visualization_marker topic
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # Timer to trigger updates every second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback) 
        
        rospy.loginfo("Waypoint Visualizer started.")

    def timer_callback(self, event):
        # Read the waypoints from the parameter server
        last_wp = rospy.get_param('/last_waypoint', [0.0, 0.0])
        current_wp = rospy.get_param('/current_waypoint', [0.0, 0.0])
        next_wp = rospy.get_param('/next_waypoint', [0.0, 0.0])

        # Create and publish markers for each waypoint
        self.publish_marker(last_wp, [1.0, 1.0, 0.0], "last_waypoint", 0)       # Yellow for last waypoint
        self.publish_marker(current_wp, [0.0, 0.0, 1.0], "current_waypoint", 1) # Blue for current waypoint
        self.publish_marker(next_wp, [1.0, 0.0, 0.0], "next_waypoint", 2)       # Red for next waypoint

    def publish_marker(self, wp, color, ns, id_):
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "odom"         # Reference frame for the markers
        marker.header.stamp = rospy.Time.now()  # Current Time Stamp
        marker.ns = ns                          # Namespace for the marker
        marker.id = id_                         # Unique ID for the marker
        marker.type = Marker.SPHERE             # Shape of the marker
        marker.action = Marker.ADD              # Add or modify the marker
        marker.scale.x = 0.3                    # Scale of the marker in x dimension
        marker.scale.y = 0.3                    # Scale of the marker in y dimension
        marker.scale.z = 0.3                    # Scale of the marker in z dimension
        marker.color.r = color[0]               # Red component of the color
        marker.color.g = color[1]               # Green component of the color
        marker.color.b = color[2]               # Blue component of the color
        marker.color.a = 1.0                    # Alpha (transparency) of the color
        marker.pose.orientation.w = 1.0         # Orientation of the marker
        marker.pose.position.x = wp[0]          # x position of the marker
        marker.pose.position.y = wp[1]          # y position of the marker
        marker.pose.position.z = 0.0            # z position of the marker

        # Publish the marker to the topic
        self.pub.publish(marker)

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'waypoint_visualizer'
    rospy.init_node('waypoint_visualizer')

    # Create an instance of the WaypointVisualizer class
    viz = WaypointVisualizer()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
