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
from mpl_toolkits.mplot3d import Axes3D # need for 3D plot
import numpy.ma as ma
import numpy as np
import yaml

import time
import rospy, rostopic
from std_msgs.msg import Float64
from passing_intention_lstm.msg import ais_info, DictionaryLSTMMsg, KeyValueLSTMMsg


OWN_ROBOT = "robot_0"
LSTM_TOPIC = "LSTM_out"
TOTAL_ROBOT = 1

class GraphVisualizer:
    def __init__(self):
        self.name = OWN_ROBOT

        # plot configure
        self.fig = plt.figure(figsize=(8,6))
        self.to_draw_ax = self.fig.add_subplot(1,1,1)
        plt.ion() # essential

        self.start_time = None
        self.lstm_msg = None
        self.lstm_sub = rospy.Subscriber(OWN_ROBOT + "/" + LSTM_TOPIC,
                                        DictionaryLSTMMsg, self.lstm_callback, queue_size=1)
        
        self._color_code = yaml.safe_load(open("../../param/color_code.yaml"))
        self.move_figure(self.fig, 910, 320)

        self.initialize_prob_array()


    def initialize_prob_array(self):
        for i in range(TOTAL_ROBOT+1):
            if "robot_{}".format(str(i)) != self.name:
                exec(
                    "self.prob_l_robot_" +
                    str(i) +
                    "= []"
                )
                exec(
                    "self.prob_r_robot_" +
                    str(i) +
                    "= []"
                )
                exec(
                    "self.time_robot_" +
                    str(i) +
                    "= []"
                )


    def lstm_callback(self, msg):
        self.lstm_msg = msg
        # print(msg)

        if self.start_time is None: # make sure time array can be built
            return

        for each_pred_msg in self.lstm_msg.items:
            robot_idx = each_pred_msg.key
            robot_idx_probability = each_pred_msg.probability

            # rospy.loginfo("key {} probability {}".format(each_pred_msg.key, each_pred_msg.probability))
            
            # l, r probability pull and append
            exec("self.prob_l_robot_" + str(robot_idx) + ".append(robot_idx_probability[0])")
            exec("self.prob_r_robot_" + str(robot_idx) + ".append(robot_idx_probability[1])")

            # time append
            current_time = rospy.Time.now().to_sec()
            diff_time = current_time - self.start_time
            exec("self.time_robot_" + str(robot_idx) + ".append(diff_time)")

            # print(eval("self.prob_l_robot_" + str(robot_idx)))
            # print(eval("self.prob_r_robot_" + str(robot_idx)))


    def move_figure(self, f, x, y):
        """Move figure's upper left corner to pixel (x, y)"""
        backend = matplotlib.get_backend()
        if backend == 'TkAgg':
            f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
        elif backend == 'WXAgg':
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
            for i in range(1, TOTAL_ROBOT + 1):
                if "robot_{}".format(str(i)) != self.name:
                    prob_l = eval("self.prob_l_robot_" + str(i))
                    prob_r = eval("self.prob_r_robot_" + str(i))
                    time_stamps = eval("self.time_robot_" + str(i))

                    color_array = [each_color / 255.0 for each_color in self._color_code["robot_{}".format(str(i))]['color_code']]
                    color_array = tuple(color_array)

                    self.to_draw_ax.plot(time_stamps, prob_l, color=color_array, label="robot_" + str(i) + "_left")
                    self.to_draw_ax.plot(time_stamps, prob_r, color=color_array, linestyle='--', label="robot_" + str(i) + "_right")
                    # print(l_ls, r_ls)

            # configure plot
            # ---------------------------------------------------
            self.to_draw_ax.set_xlabel('Operation time (sec)', fontsize=18)
            self.to_draw_ax.set_ylabel('Probability', fontsize=18)
            self.to_draw_ax.tick_params(axis='both', which='major', labelsize=14)
            self.to_draw_ax.set_ylim(0, 1.1)
            # ---------------------------------------------------

            # draw plot and clear
            plt.legend()
            plt.show()
            plt.pause(0.00001)
            self.to_draw_ax.cla()



    def spin(self):
        self.live_plot(self.lstm_msg)
        rospy.Rate(5).sleep()


def main():
    graph_visualizer = GraphVisualizer()
    # rospy.spin()

    while not rospy.is_shutdown():
        graph_visualizer.spin()


if __name__ == "__main__":
    rospy.init_node("live_plot_probability")
    rospy.sleep(1)
    main()