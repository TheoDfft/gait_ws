#!/usr/bin/env python3

import rospy
import time
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates

class GazeboPauser:
    def __init__(self):
        rospy.init_node('pause_after_spawn')
        self.model_name = "monoped"
        self.model_found = False
        self.has_paused = False
        
        # Wait for Gazebo to be fully up
        rospy.loginfo("Waiting for Gazebo services...")
        try:
            rospy.wait_for_service('/gazebo/pause_physics', timeout=20)
            rospy.loginfo("Gazebo services are available")
        except rospy.ROSException:
            rospy.logerr("Timeout waiting for Gazebo services")
            return
            
        # Subscribe to model states to detect when our model appears
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        
        # Create the service proxy for pausing
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        
        # Start a timer to check if we should pause
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        
        rospy.loginfo(f"Waiting for model '{self.model_name}' to appear...")
        rospy.spin()
    
    def model_callback(self, msg):
        """Called when model states are published"""
        if self.model_name in msg.name and not self.model_found:
            self.model_found = True
            rospy.loginfo(f"Model '{self.model_name}' has appeared in Gazebo")
            
            # Wait a bit for the model to stabilize before pausing
            rospy.Timer(rospy.Duration(1.0), self.pause_callback, oneshot=True)
    
    def pause_callback(self, event):
        """Called after a delay to pause Gazebo"""
        if not self.has_paused:
            self.pause_gazebo()
    
    def timer_callback(self, event):
        """Fallback to ensure we attempt to pause even if model callback doesn't trigger"""
        if self.model_found and not self.has_paused:
            self.pause_gazebo()
    
    def pause_gazebo(self):
        """Call the pause service with error handling"""
        try:
            self.has_paused = True
            rospy.loginfo("Attempting to pause Gazebo simulation...")
            self.pause_physics()
            rospy.loginfo("Gazebo simulation paused successfully")
            
            # We succeeded, so shut down the node gracefully
            rospy.signal_shutdown("Successfully paused Gazebo")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to pause Gazebo: {e}")
            self.has_paused = False  # Try again next time

if __name__ == '__main__':
    try:
        GazeboPauser()
    except rospy.ROSInterruptException:
        pass 