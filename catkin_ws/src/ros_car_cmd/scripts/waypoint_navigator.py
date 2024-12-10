#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

###################################################################################################

class WaypointNavigator:
    """
    Waypoint Navigator node for autonomous robot navigation.

    This node enables the robot to navigate towards a specified waypoint by processing odometry data
    and issuing velocity commands. It continuously monitors the robot's position and orientation,
    calculates the necessary adjustments, and controls the robot's movement to reach the target waypoint.
    """

    def __init__(self):
        """
        Initializes the WaypointNavigator.

        - Sets navigation parameters such as arrival threshold and base speeds.
        - Retrieves the initial current waypoint from the ROS parameter server.
        - Initializes the robot's state (position and orientation).
        - Sets up publishers and subscribers for controlling the robot and receiving odometry data.
        - Initializes a timer for periodic control loop execution.
        - Logs the start of the Waypoint Navigator.
        """
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
        """
        Callback function to handle incoming odometry messages.

        Extracts the robot's current position and orientation from the Odometry message and updates
        the internal state accordingly.

        Args:
            msg (Odometry): The odometry message containing the robot's current state.
        """
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
        """
        Periodic control loop to navigate towards the current waypoint.

        This method is called at regular intervals defined by the timer. It performs the following steps:
        - Updates the current waypoint from the parameter server to allow dynamic changes.
        - Calculates the distance and angle to the current waypoint.
        - Determines whether to stop or adjust the robot's movement based on the distance and orientation.
        - Publishes appropriate velocity commands to navigate the robot towards the waypoint.

        Args:
            event (rospy.timer.TimerEvent): Timer event information (unused).
        """
        # Update the current waypoint from the parameter server (to allow dynamic changes)
        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        
        # Calculate the distance and angle to the current waypoint
        dx = self.current_waypoint[0] - self.robot_x
        dy = self.current_waypoint[1] - self.robot_y
        dist = math.sqrt(dx*dx + dy*dy)  # Euclidean distance to the waypoint

        # If the robot is close enough to the waypoint, stop the robot
        if dist < self.arrival_threshold:
            self.stop_robot()
            rospy.loginfo("Waypoint reached. Stopping the robot.")
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
            # Adjust direction based on the sign of error_yaw
            cmd.angular.z = self.angular_speed * (error_yaw / abs(error_yaw))
            cmd.linear.x = 0.0  # No forward motion while turning
            rospy.logdebug(f"Rotating in place. Angular Z: {cmd.angular.z}")
        else:
            # The robot is well-oriented; move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            rospy.logdebug(f"Moving forward. Linear X: {cmd.linear.x}")
        
        # Publish the velocity command to the /cmd_vel topic
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """
        Stops the robot by publishing a zero velocity command.

        Sends a `Twist` message with zero linear and angular velocities to halt the robot's movement.
        """
        # Publish a Twist message with zero velocity to stop the robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Robot stopped.")

###################################################################################################

if __name__ == '__main__':
    """
    Main entry point for the WaypointNavigator node.

    - Initializes the ROS node with the name 'waypoint_navigator'.
    - Creates an instance of the WaypointNavigator class.
    - Keeps the node running to process callbacks.
    """
    # Initialize the ROS node with the name 'waypoint_navigator'
    rospy.init_node('waypoint_navigator')

    # Create an instance of the WaypointNavigator class
    navigator = WaypointNavigator()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
