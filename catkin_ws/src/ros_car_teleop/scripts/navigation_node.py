#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ros_car_msgs.msg import MoveControls, SpeedControls
        
###################################################################################################

class TeleopNode:
    """
    Teleoperation node for controlling the robot's movement through commands and set speeds.

    This node reads parameters from the ROS parameter server to initialize speed settings and waypoints.
    It subscribes to the `/teleop_cmd` topic to receive movement commands and to the `/set_speed` topic
    to update linear and angular speeds. It publishes velocity commands to the `/cmd_vel` topic using
    `Twist` messages.
    """
    def __init__(self):
        """
        Initializes the TeleopNode.

        - Reads initial speed values and waypoints from the ROS parameter server.
        - Sets up subscribers for movement commands and speed adjustments.
        - Sets up a publisher for sending velocity commands to `/cmd_vel`.
        - Logs the initial waypoint information.
        """
        # Read parameters from the parameter server
        self.current_linear_speed = rospy.get_param('/default_linear_speed', 0.5)
        self.current_angular_speed = rospy.get_param('/default_angular_speed', 1.0)

        self.current_waypoint = rospy.get_param('/current_waypoint', [0.0, 0.0])
        self.last_waypoint = rospy.get_param('/last_waypoint', [-1.0, -1.0])
        self.next_waypoint = rospy.get_param('/next_waypoint', [1.0, 1.0])

        rospy.loginfo("Current Waypoint: %s", str(self.current_waypoint))
        rospy.loginfo("Last Waypoint: %s", str(self.last_waypoint))
        rospy.loginfo("Next Waypoint: %s", str(self.next_waypoint))

        # Subscribers and Publisher for controlling the robot
        self.sub_cmd = rospy.Subscriber("/teleop_cmd", MoveControls, self.cmd_callback)
        self.sub_speed = rospy.Subscriber("/set_speed", SpeedControls, self.speed_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def speed_callback(self, msg):
        """
        Callback to update linear and angular speeds.

        Updates the current linear and angular speeds based on the received `SpeedControls` message
        and logs the updated speeds.

        Args:
            msg (SpeedControls): Message containing the new speed values.
        """
        # Update linear and angular speeds from received message
        self.current_linear_speed = msg.linear_speed
        self.current_angular_speed = msg.angular_speed

        rospy.loginfo("Speed changed: linear=%.2f, angular=%.2f" % (self.current_linear_speed, self.current_angular_speed))

    def cmd_callback(self, msg):
        """
        Callback to handle received movement commands.

        Creates and publishes a `Twist` message based on the received `MoveControls` command.

        Args:
            msg (MoveControls): Message containing the movement command.
        """
        # Create a Twist message to represent movement commands
        twist = Twist()
        
        if msg.command == MoveControls.FORWARD:
            # Move forward
            twist.linear.x = self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.BACKWARD:
            # Move backward
            twist.linear.x = -self.current_linear_speed
            twist.angular.z = 0.0
        elif msg.command == MoveControls.LEFT:
            # Turn left
            twist.linear.x = 0.0
            twist.angular.z = self.current_angular_speed
        elif msg.command == MoveControls.RIGHT:
            # Turn right
            twist.linear.x = 0.0
            twist.angular.z = -self.current_angular_speed
        elif msg.command == MoveControls.STOP:
            # Stop movement
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message to the /cmd_vel topic
        self.pub.publish(twist)

###################################################################################################

if __name__ == '__main__':
    """
    Main entry point for the TeleopNode.

    - Initializes the ROS node with the name 'teleop_node'.
    - Creates an instance of the TeleopNode class.
    - Keeps the node running to process callbacks.
    """
    # Initialize the ROS node with the name 'teleop_node'
    rospy.init_node('teleop_node')

    # Create an instance of the TeleopNode class
    node = TeleopNode()

    # Keep the node running to process callbacks
    rospy.spin()

###################################################################################################
