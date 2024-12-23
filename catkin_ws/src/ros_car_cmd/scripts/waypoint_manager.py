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

    This node manages the robot's waypoints by providing a service to get the current,
    last, and next waypoints. It generates the next waypoint randomly within the specified
    bounds of the world and updates waypoints only when the robot reaches the current one.
    """
    def __init__(self):
        """
        Initializes the WaypointManager.

        - Loads waypoints and world bounds from the parameter server.
        - Subscribes to the /odom topic to get the robot's current position.
        - Sets up a ROS service to handle waypoint requests.
        """
        # Read initial waypoints from the parameter server
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        self.last_waypoint = rospy.get_param('/last_waypoint', [-1.0, -1.0])
        self.next_waypoint = rospy.get_param('/next_waypoint', [2.0, 1.5])

        # World bounds for waypoint generation loaded from parameter server
        self.world_bounds = rospy.get_param('/world_bounds', {
            'x_min': -5.0,
            'x_max': 5.0,
            'y_min': -5.0,
            'y_max': 5.0
        })

        # Secret key for service access
        self.secret_key = rospy.get_param('~secret_key', 'default')

        # Arrival threshold to determine if the waypoint has been reached
        self.arrival_threshold = rospy.get_param('/arrival_threshold', 0.2)

        # Current position of the robot (updated by odometry)
        self.robot_position = [0.0, 0.0]

        # Subscriber to odometry topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Service to get the next waypoint
        self.waypoint_service = rospy.Service('/waypoint_request', WaypointService, self.handle_waypoint_request)

        rospy.loginfo("Waypoint Manager started.")
        rospy.loginfo("Current waypoint: %s", str(self.current_waypoint))
        rospy.loginfo("Next waypoint: %s", str(self.next_waypoint))

    def has_reached_waypoint(self):
        """
        Checks if the robot has reached the current waypoint based on the arrival threshold.

        Returns:
            bool: True if the robot is within the arrival threshold of the current waypoint.
        """
        dx = self.robot_position[0] - self.current_waypoint[0]
        dy = self.robot_position[1] - self.current_waypoint[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance < self.arrival_threshold

    def generate_next_waypoint(self):
        """
        Generates a new random waypoint within the world bounds.

        Returns:
            list: A list containing the x and y coordinates of the next waypoint.
        """
        rand_x = random.uniform(self.world_bounds['x_min'], self.world_bounds['x_max'])
        rand_y = random.uniform(self.world_bounds['y_min'], self.world_bounds['y_max'])
        return [rand_x, rand_y]
    
    def odom_callback(self, msg):
        """
        Callback function to update the robot's current position from odometry.

        Args:
            msg (Odometry): The odometry message containing the robot's position.
        """
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

    def handle_waypoint_request(self, req):
        """
        Handles requests to the /waypoint_request service.

        This service returns the current waypoint, last waypoint, and next waypoint.
        It requires a valid secret key to process the request.

        Args:
            req: The service request containing the secret key.

        Returns:
            WaypointServiceResponse: Contains last, current, and next waypoints if the key is valid.
        """
        if req.secret_key != self.secret_key:
            rospy.logwarn("Invalid secret key provided: %s", req.secret_key)
            raise rospy.ServiceException("Invalid secret key.")

        # Check if the robot has reached the current waypoint
        if self.has_reached_waypoint():
            # Update waypoints
            self.last_waypoint = self.current_waypoint
            self.current_waypoint = self.next_waypoint
            self.next_waypoint = self.generate_next_waypoint()

            rospy.loginfo("Updated Waypoints:")
            rospy.loginfo("--Last Waypoint: %s", str(self.last_waypoint))
            rospy.loginfo("--Current Waypoint: %s", str(self.current_waypoint))
            rospy.loginfo("--Next Waypoint: %s", str(self.next_waypoint))

        # Return the response
        response = WaypointServiceResponse()
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
    try:
        # Initialize the ROS node with the name 'waypoint_manager'
        rospy.init_node('waypoint_manager')

        # Create an instance of the WaypointManager class
        manager = WaypointManager()

        # Keep the node running to process callbacks
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointManager node interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")


###################################################################################################
