#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from ros_car_msgs.srv import ResetPose, ResetPoseResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

###################################################################################################

class ResetPoseServer:
    def __init__(self):
        # Retrieve the initial position parameters from the parameter server
        self.initial_x = rospy.get_param('/initial_x', 0.0)     # Default x position
        self.initial_y = rospy.get_param('/initial_y', 0.0)     # Default y position
        self.initial_yaw = rospy.get_param('/initial_yaw', 0.0) # Default yaw angle (in radians)

        # Create a ROS service to handle reset pose requests
        self.service = rospy.Service('/reset_pose', ResetPose, self.handle_reset_pose)

        # Wait for the Gazebo service to be available
        rospy.wait_for_service('/gazebo/set_model_state')

        # Create a proxy for the Gazebo service to set model states
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.loginfo("Reset Pose Service ready with initial pose x=%.2f, y=%.2f, yaw=%.2f",
                      self.initial_x, self.initial_y, self.initial_yaw)

    def handle_reset_pose(self, req):
        # Create a request to set the model state in Gazebo
        state_req = SetModelStateRequest()

        # Name of the robot model in Gazebo
        state_req.model_state.model_name = "ros_car"
        state_req.model_state.pose.position.x = self.initial_x
        state_req.model_state.pose.position.y = self.initial_y

        # Keep the robot on the ground plane
        state_req.model_state.pose.position.z = 0.0

        # Convert yaw angle to a quaternion
        quat = quaternion_from_euler(0, 0, self.initial_yaw)
        state_req.model_state.pose.orientation.x = quat[0]
        state_req.model_state.pose.orientation.y = quat[1]
        state_req.model_state.pose.orientation.z = quat[2]
        state_req.model_state.pose.orientation.w = quat[3]

        # Set the twist to zero (no velocity)
        state_req.model_state.twist = Twist()

        # Use the "world" reference frame
        state_req.model_state.reference_frame = "world"

        try:
            # Call the Gazebo service to reset the robot pose
            resp = self.set_model_state(state_req)
            if resp.success:
                rospy.loginfo("Robot pose reset successfully.")
                return ResetPoseResponse(success=True)
            else:
                rospy.logwarn("Failed to reset pose: %s", resp.status_message)
                return ResetPoseResponse(success=False)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return ResetPoseResponse(success=False)

###################################################################################################

if __name__ == '__main__':
    # Initialize the ROS node with the name 'reset_pose_server'
    rospy.init_node('reset_pose_server')

    # Create an instance of the ResetPoseServer class
    server = ResetPoseServer()
    
    # Keep the server running to process reset pose requests
    rospy.spin()

###################################################################################################
