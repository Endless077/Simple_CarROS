#!/usr/bin/env python3
import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from ros_car_msgs.msg import MoveControls, SpeedControls

# Global variables
current_linear_x = 0.0
current_angular_z = 0.0

###################################################################################################

def control_callback(msg):
    """
    Handles incoming movement control commands to update robot velocities.

    This callback function listens to movement commands and updates the global linear and angular
    velocities based on the received command. Supported commands include FORWARD, BACKWARD, LEFT,
    RIGHT, and STOP. If an unknown command is received, a warning is logged.

    Args:
        msg (MoveControls): The message containing the movement command.
    """
    global current_linear_x, current_angular_z
    if msg.command == 1:    # FORWARD
        current_linear_x = 0.5
        current_angular_z = 0.0
    elif msg.command == 2:  # BACKWARD
        current_linear_x = -0.5
        current_angular_z = 0.0
    elif msg.command == 3:  # LEFT
        current_linear_x = 0.0
        current_angular_z = 1.0
    elif msg.command == 4:  # RIGHT
        current_linear_x = 0.0
        current_angular_z = -1.0
    elif msg.command == 5:  # STOP
        current_linear_x = 0.0
        current_angular_z = 0.0
    else:
        rospy.logwarn("Unknown command received: %d", msg.command)

def speed_callback(msg):
    """
    Updates the robot's linear and angular speeds based on received speed settings.

    This callback function listens to speed setting messages and updates the global linear and
    angular velocities accordingly. It also logs the updated speeds.

    Args:
        msg (SpeedControls): The message containing the new speed settings.
    """
    # Get Global Variables
    global current_linear_x, current_angular_z
    current_linear_x = msg.linear_speed
    current_angular_z = msg.angular_speed

    rospy.loginfo("Updated speeds: Linear X = %.2f m/s, Angular Z = %.2f rad/s",
                  current_linear_x, current_angular_z)

def odom_publisher():
    """
    Initializes and runs the odometry publisher node.

    This function sets up the ROS node responsible for publishing odometry data based on
    movement commands and speed settings. It subscribes to the `/teleop_cmd` and `/set_speed`
    topics to receive control inputs and updates the robot's position and orientation accordingly.
    The odometry data is published to the `/odom` topic, and corresponding TF transforms are
    broadcasted to enable other ROS components to utilize the robot's state information.
    """
    # Init the odom_publisher node
    rospy.init_node('odom_publisher', anonymous=True)

    # Get Global variables
    global current_linear_x, current_angular_z

    # Publisher for the /odom topic
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Subscriber for directional control commands
    rospy.Subscriber('/teleop_cmd', MoveControls, control_callback)

    # Subscriber for speed settings
    rospy.Subscriber('/set_speed', SpeedControls, speed_callback)

    # TF broadcaster to publish odometry transformations
    br = tf.TransformBroadcaster()

    # Publishing rate
    rate = rospy.Rate(10)  # 10 Hz

    # Variables to track the robot's position and time
    x = 0.0
    y = 0.0
    th = 0.0
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # Get the current time and calculate the time difference
        current_time = rospy.Time.now()
        delta_t = (current_time - last_time).to_sec()
        last_time = current_time

        # Calculate changes in position and orientation based on current velocities and time elapsed
        delta_x = current_linear_x * delta_t * rospy.cos(th)
        delta_y = current_linear_x * delta_t * rospy.sin(th)
        delta_th = current_angular_z * delta_t

        x += delta_x
        y += delta_y
        th += delta_th

        # Create a quaternion from the theta angle
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # Publish the transformation from odom -> base_link
        br.sendTransform((x, y, 0),
                         odom_quat,
                         current_time,
                         "base_link",
                         "odom")

        # Create an Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = current_linear_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = current_angular_z

        # Publish the Odometry message
        odom_pub.publish(odom)

        # Optional debug log for current odometry
        rospy.logdebug("Published Odom: x=%.2f, y=%.2f, th=%.2f, lin_x=%.2f, ang_z=%.2f",
                      x, y, th, current_linear_x, current_angular_z)

        # Sleep until the next cycle
        rate.sleep()

###################################################################################################

if __name__ == '__main__':
    """
    Main entry point for the odom_publisher node.

    - Initializes and starts the odometry publisher.
    - Handles graceful shutdown on ROS interrupt exceptions.
    """
    # Init the odom Publisher
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass

###################################################################################################
