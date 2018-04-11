#!/usr/bin/python
import rospy
import rosbag
import rospkg
import sys
import numpy as np
import matplotlib.pyplot as plt
from folding_assembly_controller.msg import FoldingControllerFeedback

def load_bag(bag_name):
    bag_dir= rospkg.RosPack().get_path("folding_assembly_controller") + "/bags/" + bag_name

    bag = rosbag.Bag(bag_dir)

    period_list = []
    t_list = []

    for topic, msg, t in bag.read_messages():
        period_list += [[msg.feedback.control_period]]
        t_list += [[t.to_sec()]]

    bag.close()

    period = np.array(period_list)
    t = np.array(t_list)
    t -= t[0][0]

    return [period, t]

if __name__ == "__main__":
    if len(sys.argv) > 1:
        bag_name = sys.argv[1]
    else:
        raise Exception("Need bag name")

    [period, t] = load_bag(bag_name)

    plt.figure()
    plt.plot(t, period)
    plt.xlabel("Time [s]")
    plt.ylabel("Estimated control period [s]")
    plt.show()

    np.savetxt(bag_name + ".csv", period, delimiter=",")
