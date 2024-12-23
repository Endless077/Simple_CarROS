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
    Waypoint Navigator node for autonomous robot navigation with simple obstacle avoidance.

    This node communicates with the Waypoint Manager service to retrieve waypoints
    and autonomously navigates towards them. If an obstacle is detected in front,
    the robot stops and rotates to avoid it before continuing.
    """

    def __init__(self):
        """
        Initializes the WaypointNavigator node with parameters from the ROS parameter server.

        The node reads parameters such as linear_speed, angular_speed, arrival_threshold,
        obstacle_threshold, and front_angle_width. It sets up subscribers for the /odom and /scan topics,
        a publisher for the /cmd_vel topic, and a service proxy for the /waypoint_request service.
        """
        # Navigation parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.arrival_threshold = rospy.get_param('~arrival_threshold', 0.2)

        # Obstacle avoidance parameters
        self.front_angle_width  = rospy.get_param('~front_angle_width', 20)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Current waypoint
        self.current_waypoint = None

        # Flag for simple collision avoidance
        self.obstacle_in_front = False

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscriber for LaserScan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Waypoint service client
        rospy.wait_for_service('/waypoint_request')
        self.waypoint_request = rospy.ServiceProxy('/waypoint_request', WaypointService)

        rospy.loginfo("Waypoint Navigator with Obstacle Avoidance started.")

    def odom_callback(self, msg):
        """
        Callback for the /odom topic.

        Extracts the robot's current position (x, y) and orientation (yaw) from the Odometry message.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def scan_callback(self, scan_msg):
        """
        Callback for the /scan topic (LaserScan).

        Reads the laser data and checks for any obstacles in front of the robot within
        a given angular sector (± front_angle_width) and a certain distance (obstacle_threshold).
        Sets self.obstacle_in_front to True if an obstacle is detected.
        """
        # Convert the front angle width from degrees to radians
        half_angle = math.radians(self.front_angle_width)

        # Extract angles from the LaserScan
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max

        # A list to store distances in the front sector
        front_distances = []

        # Determine the central index for the laser (angle ~ 0.0)
        num_rays = len(scan_msg.ranges)
        # Convert angle 0.0 to an index (angle_min is the start of the scan)
        mid_index = int((0.0 - angle_min) / angle_increment)

        # Convert front_angle_width to a number of samples around mid_index
        rays_span = int(self.front_angle_width / (angle_increment * 180.0 / math.pi))

        # Ensure the indices don't go out of range
        left_i = max(0, mid_index - rays_span)
        right_i = min(num_rays - 1, mid_index + rays_span)

        # Collect distances in that front sector
        for i in range(left_i, right_i + 1):
            dist = scan_msg.ranges[i]
            # Only consider valid distances (filter out inf or NaN)
            if not math.isinf(dist) and not math.isnan(dist):
                front_distances.append(dist)

        # If any obstacle is closer than obstacle_threshold, set obstacle_in_front
        if front_distances and min(front_distances) < self.obstacle_threshold:
            self.obstacle_in_front = True
        else:
            self.obstacle_in_front = False

    def request_next_waypoint(self):
        """
        Requests the next waypoint from the Waypoint Manager service.

        Updates self.current_waypoint with the service response if successful.
        """
        try:
            req = WaypointServiceRequest()
            req.secret_key = rospy.get_param('~secret_key', 'default_secret')
            response = self.waypoint_request(req)
            self.current_waypoint = response.current_waypoint
            rospy.loginfo("New waypoint received: %s", self.current_waypoint)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call waypoint service: %s", str(e))
            self.current_waypoint = None

    def navigate_to_waypoint(self):
        """
        Main navigation logic to move the robot towards the current waypoint.

        1. If no current waypoint, request a new one.
        2. If there's an obstacle in front, call obstacle_avoidance.
        3. Otherwise, move towards the waypoint until within arrival_threshold;
           then request the next waypoint.
        """
        if self.current_waypoint is None:
            rospy.logwarn("No waypoint set. Requesting a new waypoint...")
            self.request_next_waypoint()
            return

        # If there's an obstacle, call avoidance
        if self.obstacle_in_front:
            self.obstacle_avoidance()
            return

        # Calculate the distance to the current waypoint
        dx = self.current_waypoint[0] - self.robot_x
        dy = self.current_waypoint[1] - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we've arrived
        if distance < self.arrival_threshold:
            rospy.loginfo("Reached waypoint: %s", self.current_waypoint)
            self.stop_robot()
            self.request_next_waypoint()
            return

        # Otherwise, compute heading to the waypoint
        theta_des = math.atan2(dy, dx)
        error_yaw = theta_des - self.robot_yaw
        # Normalize the angle to [-pi, pi]
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        # Create a Twist command
        cmd = Twist()
        if abs(error_yaw) > 0.1:
            # Rotate in place if yaw error is large
            cmd.angular.z = self.angular_speed * (error_yaw / abs(error_yaw))
            cmd.linear.x = 0.0
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def obstacle_avoidance(self):
        rospy.logwarn("Obstacle detected! Rotating to avoid collision...")
        rate = rospy.Rate(10)  # frequenza di controllo, es. 10 Hz

        while not rospy.is_shutdown() and self.obstacle_in_front:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotazione costante
            self.cmd_pub.publish(cmd)
            
            rate.sleep()
        
        # Appena il while esce, significa che self.obstacle_in_front è False
        rospy.loginfo("Obstacle no longer in front, resuming navigation.")
        # Ferma la rotazione
        self.stop_robot()


    def stop_robot(self):
        """
        Sends a zero velocity command to stop the robot.
        """
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Robot stopped.")

    def run(self):
        """
        Main loop of the node.
        Runs at a fixed rate (10Hz), continuously calling navigate_to_waypoint().
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.navigate_to_waypoint()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('waypoint_navigator')
    navigator = WaypointNavigator()
    navigator.run()
