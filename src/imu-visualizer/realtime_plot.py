#!/usr/bin/env python

"""
    CLASS imu_visualizer

    purpose: 
    - Class definition of 'real time plot'
"""

# essential modules
import numpy as np
import math
import matplotlib
matplotlib.use('GTKAgg') # backend choice for speed up / before pyplot
import matplotlib.pyplot as plt

import rospy
from obstacle_avoidance_ros_pkg.msg import running_time


global fig, ax1
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
# ax2 = fig.add_subplot(2,1,1)
plt.ion()
mng = plt.get_current_fig_manager()
mng.full_screen_toggle()

OPS_TIME_LIST = []
RUN_TIME_LIST = []
DIST_DICT = dict()

first_msg = False
first_stamp = 0

def live_plot(msg):
    global first_msg, first_stamp

    if not first_msg:
        first_stamp = msg.header.stamp.to_sec()
        first_msg = True
    
    timestamp = msg.header.stamp.to_sec()
    current_time = timestamp - first_stamp
    runtime = msg.run_time

    OPS_TIME_LIST.append(current_time)
    RUN_TIME_LIST.append(runtime)

    ax1.clear()
    ax1.set_xlabel('Operation time (s)', fontsize=20)
    ax1.set_ylabel('Running Time (ms)', fontsize=20)
    ax1.tick_params(axis='both', which='major', labelsize=20)
    ax1.plot(OPS_TIME_LIST, RUN_TIME_LIST, 'r')
    ax1.set_ylim(0, 400)

    plt.show()
    plt.pause(0.00001)


if __name__ == '__main__':
    rospy.init_node('live_plot')
    rospy.Subscriber('/running_time', running_time, live_plot, queue_size=1)
    rospy.spin()