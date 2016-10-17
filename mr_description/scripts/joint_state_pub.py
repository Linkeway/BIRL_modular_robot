#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['G0_finger_joint_l', 'G0_finger_joint_r', 'I1_Joint_2', 'T2_Joint_2', 'T3_Joint_2', 'T4_Joint_2', 'I5_Joint_2', 'G6_finger_joint_l', 'G6_finger_joint_r', '_G6_finger_joint_l', '_G6_finger_joint_r', '_I5_Joint_2', '_T4_Joint_2', '_T3_Joint_2', '_T2_Joint_2', '_I1_Joint_2', '_G0_finger_joint_l', '_G0_finger_joint_r']

    hello_str.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hello_str.velocity = []
    hello_str.effort = []
    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
