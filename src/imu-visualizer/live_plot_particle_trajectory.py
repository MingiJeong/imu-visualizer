#!/usr/bin/env python

"""
    live_plot
    
    purpose: 
    - live_plot for x: time, y: value
"""


# essential python modules
import matplotlib

# matplotlib.use('GTKAgg') # backend choice for speed up / before pyplot
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.projections import get_projection_class
from mpl_toolkits.mplot3d import Axes3D  # need for 3D plot
import numpy.ma as ma
import numpy as np
import yaml
from collections import deque, defaultdict


import time
import rospy, rostopic
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, OccupancyGrid


DEFAULT_ODOM_TOPIC = "odom"
TOTAL_TARGET = 10
MONITOR_SLIDING_WINDOW_SIZE = 10000


def safe_add_key_val_to_dict_deque(dict_with_deque, key, val):
    try:
        # remove a case where None sensing value is added
        if val is not None:
            # thread_lock.acquire()
            dict_with_deque[key].append(val)
            # thread_lock.release()
    except:
        pass
        # thread_lock.release()


class GraphVisualizer:
    def __init__(self):
        # plot configure
        self.fig = plt.figure(figsize=(8, 6))
        self.to_draw_ax = self.fig.add_subplot(1, 1, 1)
        plt.ion()  # essential

        self.start_time = None
        self.target_history = dict()

        self._color_code = yaml.safe_load(open("../../param/color_code.yaml"))
        self.move_figure(self.fig, 910, 320)

        # target subscriber for sensor modeling
        for i in range(TOTAL_TARGET + 1):  # target
            # deque define for sliding window
            # -----------------------------------------------------------------------
            s = "self.target_history[{}] = deque(maxlen={})".format(i, MONITOR_SLIDING_WINDOW_SIZE)
            exec(s)
            # -----------------------------------------------------------------------

            s = "def target_sub_" + str(i) + "(self, msg): \n"
            s += "\t safe_add_key_val_to_dict_deque(self.target_history, {}, (msg.pose.pose.position.x, msg.pose.pose.position.y)) \n".format(
                i
            )
            # s += "\t print('history', self.target_history) \n"

            exec(s)
            exec("setattr(GraphVisualizer, 'callback_target_" + str(i) + "', target_sub_" + str(i) + ")")

            exec(
                "rospy.Subscriber('/target_"
                + str(i)
                + "/{}', Odometry, self.callback_target_".format(DEFAULT_ODOM_TOPIC)
                + str(i)
                + ", queue_size = 1)"
            )

    def move_figure(self, f, x, y):
        """Move figure's upper left corner to pixel (x, y)"""
        backend = matplotlib.get_backend()
        if backend == "TkAgg":
            f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
        elif backend == "WXAgg":
            f.canvas.manager.window.SetPosition((x, y))
        else:
            # This works for QT and GTK
            # You can also use window.setGeometry
            f.canvas.manager.window.move(x, y)

    def live_plot(self, to_draw_data):
        if to_draw_data is not None:
            if self.start_time is None:
                self.start_time = rospy.Time.now().to_sec()
            self.to_draw_ax.clear()

            # data preparation
            for i in range(0, TOTAL_TARGET + 1):
                target_xy = eval("self.target_history.get({})".format(i))
                # print("target x_y {}".format(target_x_y))
                if len(target_xy) > 0:

                    color_array = [each_color / 255.0 for each_color in self._color_code["robot_{}".format(str(i))]["color_code"]]
                    color_array = tuple(color_array)

                    history_xy = list(target_xy)[:-1]
                    history_x = [each_history[0] for each_history in history_xy]
                    history_y = [each_history[1] for each_history in history_xy]

                    self.to_draw_ax.plot(history_x, history_y, color=color_array)
                    self.to_draw_ax.scatter(target_xy[-1][0], target_xy[-1][1], color=color_array, label="target_" + str(i))

            # configure plot
            # ---------------------------------------------------
            self.to_draw_ax.set_xlabel("Position x [m]", fontsize=18)
            self.to_draw_ax.set_ylabel("Position y [m]", fontsize=18)
            self.to_draw_ax.tick_params(axis="both", which="major", labelsize=14)
            self.to_draw_ax.set_xlim(-25, 25)
            self.to_draw_ax.set_ylim(-25, 25)
            # ---------------------------------------------------

            # draw plot and clear
            plt.legend()
            plt.show()
            plt.pause(0.00001)
            self.to_draw_ax.cla()

    def spin(self):
        self.live_plot(self.target_history)
        rospy.Rate(5).sleep()


def main():
    graph_visualizer = GraphVisualizer()
    # rospy.spin()

    while not rospy.is_shutdown():
        graph_visualizer.spin()


if __name__ == "__main__":
    rospy.init_node("live_plot_trajectory")
    rospy.sleep(1)
    main()
