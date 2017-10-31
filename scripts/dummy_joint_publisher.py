#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def joint_publisher():
    rospy.init_node('dummy_publisher')

    publisher = rospy.Publisher('/yumi/joint_command', JointState, queue_size=10)

    msg = JointState()

    msg.name = ['gripper_l_joint', 'gripper_r_joint', 'yumi_joint_1_l', 'yumi_joint_1_r', 'yumi_joint_2_l', 'yumi_joint_2_r', 'yumi_joint_3_l', 'yumi_joint_3_r', 'yumi_joint_4_l', 'yumi_joint_4_r', 'yumi_joint_5_l', 'yumi_joint_5_r', 'yumi_joint_6_l', 'yumi_joint_6_r', 'yumi_joint_7_l', 'yumi_joint_7_r']
    msg.velocity = [0, 0, -0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    print(msg)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)
        r.sleep()

if __name__ == '__main__':
    joint_publisher()
