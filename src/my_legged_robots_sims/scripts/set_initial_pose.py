#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.msg import ModelStates
import time

def wait_for_model(model_name, timeout=30):
    """Wait until the specified model appears in the Gazebo world."""
    rospy.loginfo(f"Waiting for model '{model_name}' to spawn...")
    
    # Wait for the get_world_properties service
    rospy.wait_for_service('/gazebo/get_world_properties')
    get_world_props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    
    start_time = rospy.Time.now()
    rate = rospy.Rate(2)  # Check every 0.5 seconds
    
    while not rospy.is_shutdown():
        # Check if timeout exceeded
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logerr(f"Timeout waiting for model '{model_name}'")
            return False
            
        # Get list of models in the world
        try:
            resp = get_world_props()
            if model_name in resp.model_names:
                rospy.loginfo(f"Model '{model_name}' found in Gazebo world")
                # Add an extra delay to ensure the model is fully loaded
                rospy.sleep(1.0)
                return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
            
        rate.sleep()
    
    return False

def wait_for_controllers():
    """Wait for controllers to be available."""
    rospy.loginfo("Waiting for controllers to be loaded...")
    
    # List of controllers to wait for
    controllers = [
        '/monoped/haa_joint_position_controller/command',
        '/monoped/hfe_joint_position_controller/command',
        '/monoped/kfe_joint_position_controller/command'
    ]
    
    # Wait for each controller
    for controller in controllers:
        # Check if topic exists using a temporary subscriber
        rospy.loginfo(f"Waiting for controller: {controller}")
        topic_exists = False
        retry_count = 0
        
        while not topic_exists and retry_count < 30 and not rospy.is_shutdown():
            try:
                # Try to create a subscriber - this will raise an exception if the topic doesn't exist
                # or will receive an empty message if it exists
                msg = rospy.wait_for_message(controller, Float64, timeout=1.0)
                topic_exists = True
                rospy.loginfo(f"Controller {controller} is available")
            except rospy.ROSException:
                rospy.loginfo(f"Waiting for controller {controller}, retry {retry_count+1}/30")
                retry_count += 1
                
        if not topic_exists:
            rospy.logerr(f"Controller {controller} not available after timeout")
            return False
            
    rospy.loginfo("All controllers are available")
    return True

def set_monoped_initial_pose():
    """Set the initial joint positions for the monoped robot."""
    # Initialize the ROS node
    rospy.init_node('set_monoped_initial_pose')
    
    # Define the model name
    model_name = "monoped"
    
    # Wait for the model to be spawned
    if not wait_for_model(model_name):
        rospy.logerr("Failed to find the model in Gazebo. Exiting.")
        return
    
    # Wait for controllers to be available
    if not wait_for_controllers():
        rospy.logerr("Failed to find all controllers. Exiting.")
        return
    
    try:
        # Create publishers for each joint controller
        haa_pub = rospy.Publisher('/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        hfe_pub = rospy.Publisher('/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        kfe_pub = rospy.Publisher('/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)
        
        # Define the joint positions
        joint_positions = [-0.0, 0.9818198986954192, -2.0617062761426066]
        
        # Make sure publishers are connected
        rospy.sleep(1.0)
        
        # Publish the joint positions - do this multiple times to ensure they are received
        for _ in range(5):
            rospy.loginfo("Publishing joint positions to controllers...")
            haa_pub.publish(Float64(joint_positions[0]))
            hfe_pub.publish(Float64(joint_positions[1]))
            kfe_pub.publish(Float64(joint_positions[2]))
            rospy.sleep(0.2)
        
        rospy.loginfo("Successfully sent joint position commands to controllers")
        
        # Keep the node alive
        rospy.loginfo("Initial pose set. Node will now spin...")
        rospy.spin()
        
    except Exception as e:
        rospy.logerr(f"Error setting joint positions: {e}")

if __name__ == '__main__':
    try:
        set_monoped_initial_pose()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception caught") 