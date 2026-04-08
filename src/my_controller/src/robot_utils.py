#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def spawn_position(coords):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = "B1" 
        
        # Position and Orientation
        state.pose.position.x, state.pose.position.y, state.pose.position.z = coords[0:3]
        state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = coords[3:7]
        
        # Reset movement so it doesn't drift after teleporting
        state.twist.linear.x = 0
        state.twist.angular.z = 0

        set_state(state)
        rospy.loginfo("Robot teleported to start.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
