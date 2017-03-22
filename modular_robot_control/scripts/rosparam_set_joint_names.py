#!/usr/bin/env python
# @brief :get joint names from URDF and set them to parameter server
from urdf_parser_py.urdf import URDF
import xml.dom.minidom
import rospy
from sensor_msgs.msg import JointState
from math import pi 
def get_joints():

    while not rospy.has_param("robot_description"):
        rospy.sleep(1)
        print "Can find parameter 'robot_description' for setting 'joint_names'! Waiting..."

    description = rospy.get_param("robot_description")
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    free_joints = {}
    joint_list = [] # for maintaining the original order of the joints
    dependent_joints = rospy.get_param("dependent_joints", {})

    # Find all non-fixed joints.
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'joint':
            jtype = child.getAttribute('type')
            if jtype == 'fixed':
                continue
            name = child.getAttribute('name')

            if jtype == 'continuous':
                minval = -pi
                maxval = pi
            else:
                limit = child.getElementsByTagName('limit')[0]
                minval = float(limit.getAttribute('lower'))
                maxval = float(limit.getAttribute('upper'))

            if name in dependent_joints:
                continue
            if minval > 0 or maxval < 0:
                zeroval = (maxval + minval)/2
            else:
                zeroval = 0

            joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
            free_joints[name] = joint
            joint_list.append(name)

    # joint_state = JointState()
    # joint_state.header.stamp = rospy.Time.now()

    # Add Free Joints.
    # for (name, joint) in free_joints.items():
        # joint_state.name.append(str(name))
        # joint_state.position.append(joint['value'])
        # joint_state.velocity.append(0)
        
    log = "Get joint_names:"
    for joint in joint_list:
        log += joint
        log += ';'
    print log

    return joint_list

if __name__ == '__main__':
    try:
        joint_names = get_joints()
        rospy.set_param("/joint_names",joint_names)
    except:
        rospy.logerr("can't find parameter 'robot_description'!")
    