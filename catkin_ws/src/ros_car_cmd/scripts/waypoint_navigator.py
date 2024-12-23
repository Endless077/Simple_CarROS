#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from ros_car_msgs.srv import WaypointService, WaypointServiceRequest

class WaypointNavigator:
    """
    A ROS node for autonomous robot navigation with wall-following obstacle avoidance.

    The node communicates with the Waypoint Manager service to retrieve waypoints
    and autonomously navigates towards them. If an obstacle is detected in front,
    the robot switches to wall-following mode to circumvent the obstacle before resuming navigation.
    """
    # State machine definitions
    STATE_NAVIGATING = 0
    STATE_WALL_FOLLOWING = 1

    def __init__(self):
        """
        Initializes the WaypointNavigator node with parameters from the ROS parameter server.

        Args:
            ~linear_speed (float): Default linear speed of the robot.
            ~angular_speed (float): Default angular speed of the robot.
            ~arrival_threshold (float): Distance threshold for reaching a waypoint.
            ~obstacle_threshold (float): Distance threshold for obstacle detection.
            ~wall_follow_distance (float): Desired distance to maintain from the wall.
            ~alignment_tolerance (float): Angular error tolerance for alignment.
            ~kp_angular (float): Proportional gain for angular velocity control.
            ~ki_angular (float): Integral gain for angular velocity control.
            ~kd_angular (float): Derivative gain for angular velocity control.
        """
        # Navigation parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.arrival_threshold = rospy.get_param('~arrival_threshold', 0.3)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)
        self.wall_follow_distance = rospy.get_param('~wall_follow_distance', 0.7)
        self.front_angle_width = rospy.get_param('~front_angle_width', math.pi / 6)

        # Tolerance for alignment
        self.alignment_tolerance = rospy.get_param('~alignment_tolerance', 0.2)

        # PID parameters
        self.kp_angular = rospy.get_param('~kp_angular', 1.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.0)
        self.kd_angular = rospy.get_param('~kd_angular', 0.05)

        # PID state
        self.error_yaw_integral = 0.0
        self.prev_error_yaw = 0.0

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Current waypoint
        self.current_waypoint = None

        # Obstacle avoidance parameters
        self.regions = {}
        self.obstacle_detected = False
        self.state = self.STATE_NAVIGATING

        # Alignment state
        self.aligned = False

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscriber for LaserScan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Waypoint service client
        rospy.wait_for_service('/waypoint_request')
        self.secret_key = rospy.get_param('~secret_key', 'default')
        self.waypoint_request = rospy.ServiceProxy('/waypoint_request', WaypointService)

        rospy.loginfo("Waypoint Navigator with Wall-Following Obstacle Avoidance started.")

###################################################################################################

    def request_waypoint(self):
        """
        Requests the next waypoint from the Waypoint Manager service.

        Updates the current waypoint with the service response if successful.
        """
        try:
            req = WaypointServiceRequest()
            req.secret_key = self.secret_key
            response = self.waypoint_request(req)
            self.current_waypoint = response.current_waypoint
            rospy.loginfo("New waypoint received: %s", self.current_waypoint)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call waypoint service: %s", str(e))
            self.current_waypoint = None

    def odom_callback(self, odom_msg):
        """
        Callback for the /odom topic. Extracts the robot's current position (x, y) and orientation (yaw).

        Args:
            odom_msg (Odometry): The Odometry message containing the robot's pose.
        """
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y
        orientation_q = odom_msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def scan_callback(self, scan_msg):
        """
        Callback for the /scan topic. Reads laser data and determines distances in different regions.

        Args:
            scan_msg (LaserScan): The LaserScan message containing range data.
        """
        ranges = scan_msg.ranges
        num_samples = len(ranges)
        angle_increment = scan_msg.angle_increment

        # Calculate indices for the front angle range
        center_index = num_samples // 2
        half_width = int(self.front_angle_width / angle_increment / 2)
        front_indices = ranges[center_index - half_width : center_index + half_width + 1]

        self.regions = {
            'right': min(min(ranges[:num_samples//3]), float('inf')),
            'front': min(min(front_indices), float('inf')),
            'left': min(min(ranges[2*num_samples//3:]), float('inf'))
        }
        self.obstacle_detected = self.regions['front'] < self.obstacle_threshold

    def navigate(self):
        """
        Main navigation logic to move the robot towards the current waypoint.

        Steps:
        1. If no current waypoint, request a new one.
        2. If an obstacle is detected, switch to wall-following mode.
        3. Otherwise, move towards the waypoint until within arrival_threshold.
        4. Request the next waypoint upon arrival.
        """
        if self.current_waypoint is None:
            rospy.logwarn("No waypoint set. Requesting a new waypoint...")
            self.request_waypoint()
            return

        if self.state == self.STATE_WALL_FOLLOWING:
            self.wall_following()
            return

        if self.obstacle_detected:
            rospy.logwarn("Obstacle detected! Switching to wall-following mode.")
            self.state = self.STATE_WALL_FOLLOWING
            return

        # Calculate the distance to the current waypoint
        dx = self.current_waypoint[0] - self.robot_x
        dy = self.current_waypoint[1] - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if the robot reached the waypoint
        if distance < self.arrival_threshold:
            rospy.loginfo("Reached waypoint: %s", self.current_waypoint)
            self.stop()
            self.aligned = False
            self.request_waypoint()
            return

        # Otherwise, compute heading to the waypoint
        theta_des = math.atan2(dy, dx)
        error_yaw = theta_des - self.robot_yaw

        # Normalize the angle to [-pi, pi]
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        cmd = Twist()

        # Adjust angular speed with PID control
        # (assuming 10Hz control loop)
        dt = 0.1

        self.error_yaw_integral += error_yaw * dt
        error_yaw_derivative = (error_yaw - self.prev_error_yaw) / dt
        self.prev_error_yaw = error_yaw

        angular_speed = (
            self.kp_angular * error_yaw +
            self.ki_angular * self.error_yaw_integral +
            self.kd_angular * error_yaw_derivative
        )

        # Saturate angular speed
        angular_speed = max(-self.angular_speed, min(self.angular_speed, angular_speed))

        cmd.angular.z = angular_speed

        # Allow forward motion if error_yaw is small enough
        if abs(error_yaw) < self.alignment_tolerance:
            cmd.linear.x = self.linear_speed
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def wall_following(self):
        """
        Implements wall-following behavior to avoid obstacles.

        The robot maintains a desired distance from the wall on its left or right side.
        Adjusts angular velocity based on the distance to the wall.
        """
        cmd = Twist()
        error = 0.0

        if self.regions['front'] < self.obstacle_threshold:
            # Turn away from the obstacle in front
            if self.regions['left'] > self.regions['right']:
                cmd.angular.z = self.angular_speed
                rospy.logdebug("Turning left to follow the wall.")
            else:
                cmd.angular.z = -self.angular_speed
                rospy.logdebug("Turning right to follow the wall.")
        else:
            # Follow the wall by maintaining a constant distance
            rospy.logdebug("Following the wall.")
            error = self.wall_follow_distance - self.regions['left']
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed * error

        if self.regions['front'] > self.obstacle_threshold and abs(error) < 0.1:
            rospy.logdebug("Path clear. Switching back to navigation mode.")
            self.state = self.STATE_NAVIGATING

        self.cmd_pub.publish(cmd)

    def stop(self):
        """
        Sends a zero velocity command to stop the robot.
        """
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Robot stopped.")

    def run(self):
        """
        Main loop of the node. Continuously calls navigate() at a fixed rate.

        Runs at 10Hz by default.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.navigate()
            rate.sleep()

###################################################################################################

if __name__ == '__main__':
    """
    Main entry point for the WaypointNavigator node.

    - Initializes the ROS node with the name 'waypoint_navigator'.
    - Creates an instance of the WaypointNavigator class.
    - Keeps the node running to process callbacks.
    """
    try:
        # Initialize the ROS node with the name 'waypoint_navigator'
        rospy.init_node('waypoint_navigator')

        # Create an instance of the WaypointNavigator class
        navigator = WaypointNavigator()

        # Run the main logic of the WaypointNavigator class
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointNavigator node interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in WaypointNavigator: {e}")


###################################################################################################
