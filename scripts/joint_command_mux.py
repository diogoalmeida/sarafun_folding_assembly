#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import IPython

class JointSubscriber:
    '''Receive joint commands and verify their validity. Store independent velocity references for yumi controllers.'''
    def __init__(self, joint_topic, joint_names, max_wait_time = 1.0):
        self.joint_names = joint_names
        self.joint_data = None
        self.last_time = rospy.get_time()
        self.max_wait_time = max_wait_time
        rospy.Subscriber(joint_topic, JointState, self.joint_callback)

    def joint_callback(self, joint_data):
        '''Receive a joint command in the form of a JointState message.'''

        for own_name in self.joint_names:
            for received_name in joint_data.name:
                if own_name == received_name:
                    break
            else:
                rospy.logerr("Received joint data without values for all the configured joints. Joint data:\n")
                print(joint_data)
                return

        self.last_time = rospy.get_time()
        self.joint_data = joint_data

    def get_joint_commands(self):
        if self.joint_data == None or (rospy.get_time() - self.last_time) > self.max_wait_time:
            joint_commands = {value:0 for value in self.joint_names}
        else:
            joint_commands = {value.name:Float64(value.velocity) for value in self.joint_data if value.name in self.joint_names}

        return joint_commands


def joint_publisher():
    '''At a pre-defined rate, publish joint commands to the yumi controllers.'''
    rospy.init_node('yumi_joint_mux')

    if rospy.has_param('~controllers'):
        joint_controllers = rospy.get_param('~controllers')
    else:
        rospy.logerr("Missing joint controllers definition (controllers)")
        return False

    # TODO: sanity check
    controller_names = [key for key in joint_controllers]
    joint_names = [joint_controllers[key] for key in joint_controllers]

    publishers = {joint_controllers[key]:rospy.Publisher('/yumi/' + key + '/command', Float64, queue_size=10) for key in joint_controllers}

    if rospy.has_param('~joint_topic'):
        joint_topic = rospy.get_param('~joint_topic')
    else:
        rospy.logerr("Missing joint topic name (joint_topic)")
        return False

    if rospy.has_param('~pub_rate'):
        pub_rate = rospy.get_param('~pub_rate')
    else:
        rospy.logwarn("Missing pub rate. Using default (pub_rate)")
        pub_rate = 100

    joint_subscriber = JointSubscriber(joint_topic, joint_names)

    r = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        commands = joint_subscriber.get_joint_commands()

        for name in commands:
            publishers[name].publish(commands[name])

        r.sleep()

if __name__ == '__main__':
    joint_publisher()
