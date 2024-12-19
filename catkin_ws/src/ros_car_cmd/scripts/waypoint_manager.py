#!/usr/bin/env python3
import math
import rospy
import random
from nav_msgs.msg import Odometry
from ros_car_msgs.srv import WaypointService, WaypointServiceResponse

###################################################################################################

class WaypointManager:
    """
    Waypoint Manager node for handling robot navigation waypoints.

    This node manages the robot's waypoints by monitoring its position through odometry data.
    When the robot reaches the current waypoint within a specified threshold, the node updates
    the last, current, and next waypoints. The next waypoint is randomly generated near the
    current waypoint within the specified bounds of the world.
    It also provides a service to get the next waypoint, requiring a secret key for access.
    """
    def __init__(self):
        """
        Initializes the WaypointManager.
        """
        # Read initial waypoints from the parameter server
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        self.last_waypoint = rospy.get_param('/last_waypoint', [-1.0, -1.0])
        self.next_waypoint = rospy.get_param('/next_waypoint', [2.0, 1.5])

        # Distance threshold to determine if the robot has reached the current waypoint
        self.arrival_threshold = 0.2

        # World bounds for waypoint generation loaded from parameter server
        self.world_bounds = rospy.get_param('/world_bounds', {
            'x_min': -5.0,
            'x_max': 5.0,
            'y_min': -5.0,
            'y_max': 5.0
        })

        # Secret key for service access
        self.secret_key = rospy.get_param('~secret_key', 'default_secret')

        # Subscriber to the /odom topic to monitor the robot's position
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Service to get the next waypoint
        self.waypoint_service = rospy.Service('/waypoint_service', WaypointService, self.handle_waypoint_request)

        rospy.loginfo("Waypoint Manager started.")
        rospy.loginfo("Current waypoint: %s", str(self.current_waypoint))
        rospy.loginfo("Next waypoint: %s", str(self.next_waypoint))

    def odom_callback(self, msg):
        """
        Callback function to handle incoming odometry messages.

        This method extracts the robot's current position from the Odometry message and calculates
        the distance to the current waypoint. If the robot is within the arrival threshold,
        it updates the waypoints accordingly.

        Args:
            msg (Odometry): The odometry message containing the robot's current state.
        """
        # Extract the robot's current position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate the distance to the current waypoint
        dx = x - self.current_waypoint[0]
        dy = y - self.current_waypoint[1]
        dist = math.sqrt(dx*dx + dy*dy)

        rospy.logdebug("Current Position: (%.2f, %.2f)", x, y)
        rospy.logdebug("Current Waypoint: (%.2f, %.2f)", self.current_waypoint[0], self.current_waypoint[1])
        rospy.logdebug("Distance to Waypoint: %.2f", dist)

        # If the robot is within the arrival threshold, update the waypoints
        if dist < self.arrival_threshold:
            rospy.loginfo("Reached current waypoint: %s", str(self.current_waypoint))
            
            # Update the waypoints
            self.last_waypoint = self.current_waypoint
            self.current_waypoint = self.next_waypoint

            # Generate a new random waypoint within the world bounds
            rand_x = random.uniform(self.world_bounds['x_min'], self.world_bounds['x_max'])
            rand_y = random.uniform(self.world_bounds['y_min'], self.world_bounds['y_max'])
            self.next_waypoint = [rand_x, rand_y]

            rospy.loginfo("New current waypoint: %s", str(self.current_waypoint))
            rospy.loginfo("New next waypoint: %s", str(self.next_waypoint))

            # Update the parameters on the parameter server
            rospy.set_param('/last_waypoint', self.last_waypoint)
            rospy.set_param('/current_waypoint', self.current_waypoint)
            rospy.set_param('/next_waypoint', self.next_waypoint)

    def handle_waypoint_request(self, req):
        """
        Handles requests to the /get_next_waypoint service.

        This service returns the current waypoint, last waypoint, and next waypoint.
        It requires a valid secret key to process the request.

        Args:
            req: The service request containing the secret key.

        Returns:
            GetNextWaypointResponse: Contains last, current, and next waypoints if the key is valid.
        """
        if req.secret_key != self.secret_key:
            rospy.logwarn("Invalid secret key provided: %s", req.secret_key)
            raise rospy.ServiceException("Invalid secret key.")

        response = WaypointService()
        response.last_waypoint = self.last_waypoint
        response.current_waypoint = self.current_waypoint
        response.next_waypoint = self.next_waypoint
        return response

###################################################################################################

if __name__ == '__main__':
    """
    Main entry point for the WaypointManager node.

    - Initializes the ROS node with the name 'waypoint_manager'.
    - Creates an instance of the WaypointManager class.
    - Keeps the node running to process callbacks.
    """
    # Initialize the ROS node with the name 'waypoint_manager'
    rospy.init_node('waypoint_manager')

    # Create an instance of the WaypointManager class
    manager = WaypointManager()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
