#!/usr/bin/env python3
import rospy
from robo_car_msgs.srv import ResetPose, ResetPoseResponse

class ResetPoseServer:
    def __init__(self):
        # Create the service server
        self.service = rospy.Service('/reset_pose', ResetPose, self.handle_reset_pose)

        # Store the initial position (in the future, this can be read from parameters or URDF)
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        rospy.loginfo("Reset Pose Service ready.")

    def handle_reset_pose(self, req):
        # TODO: Implement the logic to reset the robot's pose to its initial state.
        # (for now, just log the action, but when integrating with Gazebo, teleport commands will be used)
        rospy.loginfo("Resetting robot pose to x=%.2f, y=%.2f, yaw=%.2f" % (self.initial_x, self.initial_y, self.initial_yaw))

        # TODO: When integrated with Gazebo
        # (for now, just log and return success=True):
        #   - Call a Gazebo service like /gazebo/set_model_state
        #   - Set the robot's position
        return ResetPoseResponse(success=True)


if __name__ == '__main__':
    # Initialize the ROS node with the name 'reset_pose_server'
    rospy.init_node('reset_pose_server')

    # Create an instance of the ResetPoseServer class
    server = ResetPoseServer()

    # Keep the server running to handle service requests
    rospy.spin()
