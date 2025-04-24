#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from xpp_msgs.msg import RobotStateJoint
from std_srvs.srv import Empty

class JointPub(object):
    def __init__(self, required_messages=100):
        # Number of messages required before unpausing
        self.required_messages = 10
        # Counter for received messages
        self.message_count = 0

        self.publishers_array = []
        self._haa_joint_pub = rospy.Publisher('/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        self._hfe_joint_pub = rospy.Publisher('/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        self._kfe_joint_pub = rospy.Publisher('/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)
        
        self.publishers_array.append(self._haa_joint_pub)
        self.publishers_array.append(self._hfe_joint_pub)
        self.publishers_array.append(self._kfe_joint_pub)
        
        # Flag to track if we've received enough messages and unpaused
        self.simulation_unpaused = False
        
        # Subscribe to the /xpp/joint_mono_des topic
        self.joint_des_sub = rospy.Subscriber('/xpp/joint_mono_des', RobotStateJoint, self.joint_des_callback)
        
        # Store the latest desired joint angles
        self.desired_joints = [0.0, 0.0, 0.0]
        
        # Gazebo unpause service
        rospy.loginfo("Waiting for /gazebo/unpause_physics service...")
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        rospy.loginfo("Service /gazebo/unpause_physics is available")
        rospy.loginfo(f"Will unpause after receiving {self.required_messages} joint state messages")

    def joint_des_callback(self, msg):
        # Extract joint positions from the message
        joint_positions = msg.joint_state.position
        
        # Only update if we received valid joint positions
        if len(joint_positions) >= 3:
            self.desired_joints = joint_positions[:3]  # Take the first 3 joints
            
            # Increment the message counter
            self.message_count += 1
            
            # If we haven't unpaused yet, check if we've received enough messages
            if not self.simulation_unpaused:
                rospy.loginfo(f"Received message {self.message_count}/{self.required_messages}")
                
                if self.message_count >= self.required_messages:
                    rospy.loginfo(f"Received {self.required_messages} messages! Unpausing Gazebo...")
                    try:
                        self.unpause_gazebo()
                        self.simulation_unpaused = True
                        rospy.loginfo("Gazebo unpaused. Starting to publish joint commands.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to unpause Gazebo: %s", e)
            
            # Only publish joint commands if we've unpaused
            if self.simulation_unpaused:
                rospy.loginfo("Received desired joint angles: %s", str(self.desired_joints))
                # Publish the new joint positions
                self.move_joints(self.desired_joints)

    def move_joints(self, joints_array):
        for i, publisher_object in enumerate(self.publishers_array):
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)

    def start_loop(self, rate_value = 50.0):
        rospy.loginfo(f"Start Loop - Waiting for {self.required_messages} messages on /xpp/joint_mono_des before unpausing Gazebo")
        rate = rospy.Rate(rate_value)
        
        # Main loop - just keep the node alive
        while not rospy.is_shutdown():
            # We only publish when we receive messages in the callback
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    # Get the required number of messages from ROS parameter, default to 5
    required_messages = rospy.get_param('~required_messages', 5)
    joint_publisher = JointPub(required_messages)
    rate_value = 50.0
    joint_publisher.start_loop(rate_value)
