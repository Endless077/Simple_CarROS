#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from ros_car_msgs.srv import WaypointService, WaypointServiceRequest

###################################################################################################

class WaypointNavigator:
    """
    Waypoint Navigator node for autonomous robot navigation.

    This node communicates with the Waypoint Manager service to retrieve waypoints
    and autonomously navigates towards them.
    """

    def __init__(self):
        """
        Initializes the WaypointNavigator.
        """
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)               # Base linear speed
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)             # Base angular speed
        self.arrival_threshold = rospy.get_param('~arrival_threshold', 0.2)     # Distance threshold
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Current target waypoint
        self.current_waypoint = None

        # Publisher for /cmd_vel topic to control the robot's velocity
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to /odom topic to receive the robot's odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Proxy for the Waypoint Manager service
        rospy.wait_for_service('/waypoint_request')
        self.waypoint_request = rospy.ServiceProxy('/waypoint_request', WaypointService)

        rospy.loginfo("Waypoint Navigator started.")

    def odom_callback(self, msg):
        """
        Callback function to handle incoming odometry messages.

        Args:
            msg (Odometry): The odometry message containing the robot's position and orientation.
        """
        # Extract the robot's current position and orientation
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def request_next_waypoint(self):
        """
        Requests the next waypoint from the Waypoint Manager service.
        """
        try:
            # Create a request object
            req = WaypointServiceRequest()
            req.secret_key = rospy.get_param('~secret_key', 'default_secret')

            # Call the service
            response = self.waypoint_request(req)

            # Update the current waypoint
            self.current_waypoint = response.current_waypoint
            rospy.loginfo("New waypoint received: %s", self.current_waypoint)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call waypoint service: %s", str(e))
            self.current_waypoint = None

    def navigate_to_waypoint(self):
        """
        Navigates the robot towards the current waypoint.
        """
        if self.current_waypoint is None:
            rospy.logwarn("No waypoint set. Requesting a new waypoint...")
            self.request_next_waypoint()
            return

        dx = self.current_waypoint[0] - self.robot_x
        dy = self.current_waypoint[1] - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if the waypoint has been reached
        if distance < self.arrival_threshold:
            rospy.loginfo("Reached waypoint: %s", self.current_waypoint)
            self.stop_robot()
            self.request_next_waypoint()
            return

        # Calculate angle to the waypoint
        theta_des = math.atan2(dy, dx)
        error_yaw = theta_des - self.robot_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))  # Normalize

        # Create a Twist message for movement
        cmd = Twist()

        if abs(error_yaw) > 0.1:
            # Rotate towards the waypoint
            cmd.angular.z = self.angular_speed * (error_yaw / abs(error_yaw))
            cmd.linear.x = 0.0
        else:
            # Move towards the waypoint
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        # Publish the velocity command
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocity.
        """
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Robot stopped.")

    def run(self):
        """
        Main loop to continuously navigate to waypoints.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.navigate_to_waypoint()
            rate.sleep()

###################################################################################################

if __name__ == '__main__':
    rospy.init_node('waypoint_navigator')
    navigator = WaypointNavigator()
    navigator.run()

###################################################################################################
