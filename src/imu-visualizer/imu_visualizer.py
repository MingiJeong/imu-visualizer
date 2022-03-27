#!/usr/bin/env python

"""
    CLASS imu_visualizer

    purpose: 
    - Class definition of 'IMUVisualizer' to be used in Visualizer node
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



def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "plane"
    t.child_frame_id = "imu_link"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    print("here")

    br.sendTransform(t)

if __name__ == '__main__':
      rospy.init_node('tf_broadcaster_imu')
      rospy.Subscriber('/robot_0/odom', Odometry, handle_imu_pose)
      rospy.spin()