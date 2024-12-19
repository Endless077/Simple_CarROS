#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

###################################################################################################

class WaypointVisualizer:
    """
    Waypoint Visualizer node for displaying robot waypoints in RViz.

    This node publishes visualization markers for the last, current, and next waypoints
    of the robot. It retrieves waypoint coordinates from the ROS parameter server and
    updates the markers at a regular interval.
    """
    def __init__(self):
        """
        Initializes the WaypointVisualizer.

        - Sets up a publisher to the `/visualization_marker` topic for publishing Marker messages.
        - Initializes a timer to trigger the `timer_callback` method every second.
        - Logs the start of the Waypoint Visualizer.
        """
        # Publisher to the /visualization_marker topic
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # Timer to trigger updates every second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback) 
            
        rospy.loginfo("Waypoint Visualizer started.")

    def timer_callback(self, event):
        """
        Timer callback to update and publish waypoint markers.

        This method is called periodically based on the timer interval.
        It reads the latest waypoint coordinates from the ROS parameter server
        and publishes corresponding visualization markers for each waypoint.

        Args:
            event (rospy.timer.TimerEvent): Timer event information (unused).
        """
        # Read the waypoints from the parameter server
        last_wp = rospy.get_param('/last_waypoint', [0.0, 0.0])
        current_wp = rospy.get_param('/current_waypoint', [0.0, 0.0])
        next_wp = rospy.get_param('/next_waypoint', [0.0, 0.0])

        # Create and publish markers for each waypoint
        self.publish_marker(last_wp, [1.0, 1.0, 0.0], "last_waypoint", 0)       # Yellow for last waypoint
        self.publish_marker(current_wp, [0.0, 0.0, 1.0], "current_waypoint", 1) # Blue for current waypoint
        self.publish_marker(next_wp, [1.0, 0.0, 0.0], "next_waypoint", 2)       # Red for next waypoint

    def publish_marker(self, wp, color, ns, id_):
        """
        Creates and publishes a visualization marker for a given waypoint.

        This method constructs a `Marker` message with specified properties such as
        position, color, and namespace, and publishes it to the `/visualization_marker` topic.

        Args:
            wp (list of float): The [x, y] coordinates of the waypoint.
            color (list of float): The [r, g, b] color values for the marker.
            ns (str): Namespace for the marker to categorize it.
            id_ (int): Unique identifier for the marker within its namespace.
        """
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "odom"         # Reference frame for the markers
        marker.header.stamp = rospy.Time.now()  # Current Time Stamp
        marker.ns = ns                          # Namespace for the marker
        marker.id = id_                         # Unique ID for the marker
        marker.type = Marker.SPHERE             # Shape of the marker
        marker.action = Marker.ADD              # Add or modify the marker
        marker.scale.x = 0.1                    # Scale of the marker in x dimension
        marker.scale.y = 0.1                    # Scale of the marker in y dimension
        marker.scale.z = 0.1                    # Scale of the marker in z dimension
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
    """
    Main entry point for the WaypointVisualizer node.

    - Initializes the ROS node with the name 'waypoint_visualizer'.
    - Creates an instance of the WaypointVisualizer class.
    - Keeps the node running to process callbacks.
    """
    # Initialize the ROS node with the name 'waypoint_visualizer'
    rospy.init_node('waypoint_visualizer')

    # Create an instance of the WaypointVisualizer class
    viz = WaypointVisualizer()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
