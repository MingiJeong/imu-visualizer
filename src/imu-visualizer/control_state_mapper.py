#!/usr/bin/env python

"""
    CLASS control_state_mapper

    purpose: 
    - Class definition of 'ControlStateAmpper' to be used in Visualizer node
"""

# essential modules
import numpy as np
import math

import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Temperature, Imu
from nav_msgs.msg import Odometry
from obstacle_avoidance_ros_pkg.msg import control_state


state_dict = {
    "WP_FOLLOW": 0,
    "NORMAL_AVOIDANCE": 1,
    "CONTINGENCY_AVOIDANCE": 2,
    "IDLE": 3,
}

def convert_control_state(msg):
    print("here")
    # msg_converted = control_state()
    # msg_converted.header.stamp = msg.header.stamp
    # msg_converted.header.frame_id = msg.header.frame_id

    # msg_converted.motion_state = msg.mostion_state
    # msg_converted.motion_state_int = state_dict[msg.mostion_state]
    # msg_converted.intended_course = msg.intended_course
    # msg_converted.target_heading = math.radians(msg.target_heading)
    # msg_converted.target_velocity = msg.target_velocity
    # msg_converted.current_wp_no = msg.current_wp_no
    # msg_converted.next_wp = msg.next_wp

    # pub.publish(msg_converted)


if __name__ == '__main__':
    rospy.init_node('control_state_converter')
    # pub = rospy.Publisher('robot_0/control_state_remap', control_state, queue_size=1)

    rospy.Subscriber('/robot_0/control_state', control_state, convert_control_state)
    rospy.spin()