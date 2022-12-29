#!/usr/bin/env python
# Lucas Walter
# make a joint exactly what the command wants it to be- this only works
# for position control.

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class CommandToJointState:
    def __init__(self):
        self.global_sub = rospy.rospy.Subscriber("mavros/global_position/local", Odometry, self.tf_callback, queue_size=5)
        self.joint_name = rospy.get_param("~joint_name")
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.command_sub = rospy.Subscriber("command", Float64,
                                            self.command_callback, queue_size=5)

    def command_callback(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def tf_callback(self, msg):


if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()