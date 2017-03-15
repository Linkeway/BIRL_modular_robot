#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import math
from sensor_msgs.msg import JointState
import rospy
import string
from urdf_parser_py.urdf import URDF
import xml.dom.minidom
from math import pi
import rospkg
import sys

max_vel = 0 #模块最大速度是60度每秒，一般运行速度为30度每秒
Points = [] #存储[P1 24.636 ,61.687 ,-10.713 ,180.000 ,50.974]
MotionPara = [] #存储[MOVJ P1, 30, Z1]
eps = 10/180.0*math.pi   # 
js= JointState()

def get_joints():
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

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    # Add Free Joints.
    for (name, joint) in free_joints.items():
        joint_state.name.append(str(name))
        joint_state.position.append(joint['value'])
        joint_state.velocity.append(0)

    return joint_list

def Readfile(path):
    #pack = rospkg.RosPack()
    #path = pack.get_path('modular_robot_control')+'/mrl/智造.mrl'
    f = open(path)   # 返回一个文件对象  
    line = f.readline()
    while line: 
        temp1 = line.replace(',','')    #删除逗号
        temp2 = temp1.replace(';','')   #删除分号
        line = temp2.replace('=',' ')  #用空格替代等号
        if line.startswith('P'):    #判断字符串首字母是否为P
            s = line.split()    #字符串按照空格分开，split()默认以空格分开 
            s[1] = string.atof(s[1])    #字符串转化为浮点数
            s[1] = math.radians(s[1]) #角度转弧度
            s[2] = string.atof(s[2])    #字符串转浮点数
            s[2] = s[2] - 90    #补偿，，操作臂竖直时，原系统定义此T模块为90度；而现系统定义为0度
            s[2] = math.radians(s[2])
            s[3] = string.atof(s[3])
            s[3] = s[3] - 90
            s[3] = math.radians(s[3])
            s[4] = string.atof(s[4])
            s[4] = math.radians(s[4])
            s[5] = string.atof(s[5])
            s[5] = math.radians(s[5])
            Points.append(s)    #在列表末尾添加对象
        elif line.startswith('M'):  #判断字符串首字母是否为M 
            s = line.split()
            s[2] = s[2].replace('V','')
            s[2] = string.atof(s[2])
            s[2] = s[2]*0.01 #V30代表以设定速度的百分之三十来运行
            MotionPara.append(s)
        line = f.readline()  
    f.close() 

def wait_for_robot(command_pos,actual_pos):
    reached = False
    while not reached:
         reached = True
         for i in range(len(command_pos)):
            if math.fabs(command_pos[i]-actual_pos[i]) > eps:
                 reached = False

def callback(data):
    js = data

def mrl_interpreter(simulation): #模块机器人语言解释器,使用实际机器人时
    if (simulation == 'true'):
        pub = rospy.Publisher('joint_states',JointState, queue_size=10)    #生产Publisher
        nh = rospy.init_node('mrl_interpretor', anonymous=True) #初始化node
        rate = rospy.Rate(10) # 10hz
    elif (simulation == 'false'):
        pub = rospy.Publisher('joint_command',JointState, queue_size=10) 
        sub = rospy.Subscriber('joint_states', JointState, callback) #生成Subscriber
        nh = rospy.init_node('mrl_interpretor', anonymous=True) #初始化node
        while sub.get_num_connections() < 1:
             None 
        
    msg = JointState()  #生成msg对象
    robot = URDF.from_parameter_server() 
    msg.header.frame_id = robot.get_root() #"base_link" 
    msg.name = get_joints() 
    num_joints= len(msg.name)
    i = 0
    while not rospy.is_shutdown():
        msg.header.seq = i
        msg.position.append(Points[i][1])
        msg.position.append(Points[i][2])
        msg.position.append(Points[i][3]) 
        msg.position.append(Points[i][4])
        msg.position.append(Points[i][5])
        msg.position.append(0)

        msg.velocity.append(max_vel*MotionPara[i][2])
        msg.velocity.append(max_vel*MotionPara[i][2])
        msg.velocity.append(max_vel*MotionPara[i][2])
        msg.velocity.append(max_vel*MotionPara[i][2])
        msg.velocity.append(max_vel*MotionPara[i][2])
        msg.velocity.append(0)
 
        msg.header.stamp = rospy.Time.now()

        rospy.loginfo(msg)
        
        pub.publish(msg)
        if (simulation == 'false'):
            wait_for_robot(msg.position,js.position)

        i = i + 1    
        del msg.position[5]
        del msg.position[4] 
        del msg.position[3]
        del msg.position[2]
        del msg.position[1]
        del msg.position[0]
        
        del msg.velocity[5]
        del msg.velocity[4]
        del msg.velocity[3]
        del msg.velocity[2]
        del msg.velocity[1]
        del msg.velocity[0]
        
        if (simulation == 'true'):
            rate.sleep()
 def robot_stop():
        pub = rospy.Publisher('joint_states',JointState, queue_size=10)    #生产Publisher
        stop_msg = JointState()
        for i in range(len(stop_msg)):
            stop_msg.velocity[i]=0
            pub.publish(stop_msg)
if __name__ == '__main__':
    try:
        max_vel = math.radians(30)
        #if len(sys.argv) < 2 :
        #    print 'Please input a mrl file path.'
        #    print 'E.g.: ./mrl_interpretor.py /mrl/***.mrl [max velocity in radians] [simulation]'
        #if len(sys.argv)>2:
        #    max_vel = string.atof(sys.argv[2])
            
        #Readfile(sys.argv[1])
        #mrl_interpreter(sys.argv[3])

        Readfile('../mrl/智造.mrl')
        mrl_interpreter('false')
    except rospy.ROSInterruptException:
        pass
    if sys.argv[3] == 'false':
        robot_stop()