#!/usr/bin/env python3
import os
import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

###################################################################################################

def file_paths(directory):
    """
    Get a list of file paths in the given directory.

    Parameters:
    directory (str): Path to the directory.

    Returns:
    list: List of file paths.
    """
    try:
        # Verify that the directory exists
        if not os.path.isdir(directory):
            raise ValueError(f"Directory does not exist: {directory}")
        
        # Get the complete list of files
        file_paths = [
            os.path.join(directory, file)
            for file in os.listdir(directory)
            if os.path.isfile(os.path.join(directory, file))
        ]
        return file_paths
    except Exception as e:
        print(f"Error: {e}")
        return []

def spawn_random_obstacles(max_obstacles=10, model_paths=None):
    """
    Spawns a random number of obstacles with random sizes and models in a Gazebo simulation.

    Parameters:
    max_obstacles (int): The maximum number of obstacles to spawn.
    model_paths (list): List of paths to SDF or URDF model files.

    Returns:
    None
    """
    # Check if at least one model path has been provided
    if not model_paths or len(model_paths) == 0:
        rospy.logerr("No model paths provided. Cannot spawn obstacles.")
        return

    # Initialize the ROS node
    rospy.init_node('spawn_random_obstacles')

    # Wait for the SpawnModel service to become available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        # Create a service proxy for spawning models in Gazebo
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Generate a random number of obstacles between 1 and max_obstacles
        for i in range(random.randint(1, max_obstacles)):
            # Select a random model from the provided paths
            model_path = random.choice(model_paths)
            with open(model_path, "r") as f:
                model_xml = f.read()

            # Define a random pose for the obstacle
            pose = Pose()
            pose.position.x = random.uniform(-5, 5)     # Random X position
            pose.position.y = random.uniform(-5, 5)     # Random Y position
            pose.position.z = 0                         # Static Z position
            
            # Generate random scaling values
            # (scale between 0.5x and 2.0x)
            scale_factor = random.uniform(0.5, 2.0)
            
            # Add scale to the model XML if supported
            model_xml = model_xml.replace("<scale>1 1 1</scale>", f"<scale>{scale_factor} {scale_factor} {scale_factor}</scale>")
            
            model_name = f"obstacle_{i}"  # Unique name for each obstacle
            
            # Call the service to spawn the model in Gazebo
            spawn_model(
                model_name=model_name,
                model_xml=model_xml,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world"
            )

            rospy.loginfo(f"Spawned: {model_name} with scale {scale_factor}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

###################################################################################################

if __name__ == "__main__":
    try:
        # Find the absolute path of the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Construct the absolute path to the obstacles directory
        obstacles_dir = os.path.join(current_dir, "../worlds/obstacles/")

        # Get the list of model paths (SDF files with <scale> tag)
        model_paths = file_paths(obstacles_dir)

        # Run the function to spawn random obstacles
        spawn_random_obstacles(max_obstacles=10, model_paths=model_paths)
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exception (e.g., when the node is shut down)
        rospy.loginfo("Node terminated.")
    except Exception as e:
        # Catch any other unexpected exceptions
        rospy.logerr(f"An unexpected error occurred: {e}")

###################################################################################################
