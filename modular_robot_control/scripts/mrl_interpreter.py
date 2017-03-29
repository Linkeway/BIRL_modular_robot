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
point_num = 0
MotionPara = [] #存储[MOVJ P1, 30, Z1]
eps = (0.5/180.0)*math.pi   # 
eps1 = 0.008
js = JointState()
pub = None
num_joints = 0

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

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    # Add Free Joints.
    for (name, joint) in free_joints.items():
        joint_state.name.append(str(name))
        joint_state.position.append(joint['value'])
        joint_state.velocity.append(0)

    log = "Get joint_names:"
    for joint in joint_list:
        log += joint
        log += ';'
    print log
    
    return joint_list

def Readfile(path):
    global point_num
    global MotionPara
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
            s[0:1] = []
            s.append(0) #
            Points.append(s)    #在列表末尾添加对象
            point_num +=1
        elif line.startswith('M'):  #判断字符串首字母是否为M 
            s = line.split()
            s[2] = s[2].replace('V','')
            s[2] = string.atof(s[2])
            s[2] = s[2]*0.01 #V30代表以设定速度的百分之三十来运行
            s[0:2] = []
            s[1:2] = []
            MotionPara.append(s)
        line = f.readline()  
    f.close() 

def robot_reached(command_pos,actual_pos):
    reached = True
    for i in range(len(actual_pos)):
        if math.fabs(command_pos[i]-actual_pos[i]) > eps:
            reached = False
    return reached

def callback(data):
    global js
    js = data

def mrl_interpreter(simulation):
    global num_joints 
    if (simulation == 'true'):
        pub = rospy.Publisher('joint_states',JointState, queue_size=10)    #生产Publisher
        nh = rospy.init_node('mrl_interpretor', anonymous=True) #初始化node
        rate = rospy.Rate(10) # 10hz
        rospy.sleep(rospy.Duration(8))

    elif (simulation == 'false'):
        pub = rospy.Publisher('joint_command',JointState, queue_size=10) 
        sub = rospy.Subscriber('joint_states', JointState, callback) #生成Subscriber
        nh = rospy.init_node('mrl_interpretor', anonymous=True) #初始化node
        rate = rospy.Rate(50) # 10hz
       
        while sub.get_num_connections() < 1:
             None 

    msg = JointState()  #生成msg对象
    robot = URDF.from_parameter_server() 
    msg.header.frame_id = robot.get_root() #"base_link" 
    msg.name = get_joints() 
    #msg.name.append('g6_Joint1')


    num_joints= len(msg.name)
    i = 0
    while not rospy.is_shutdown() and i < point_num:
        msg.header.seq = i

        for j in range(num_joints):
            msg.position.append(Points[i][j])
        
            #msg.position.append(0)
        angel_dif=[]
        
        if i == 0: 
            if (simulation == 'false'):
                for k in range(num_joints-1):
                    angel_dif.append(abs(Points[i][k]-js.position[k]))
                angel_dif.append(0)
            elif (simulation == 'true'): 
                for k in range(num_joints):   
                    angel_dif.append(abs(Points[i][k]-0))
        elif i > 0:
            for k in range(num_joints):    
                angel_dif.append(abs(Points[i][k]-Points[i-1][k]))
       
        max_angel = max(angel_dif)
        if max_angel < eps1:
            del msg.position[:]
            i += 1
            continue
        run_time = max_angel/max_vel
        for j in range(num_joints):
            msg.velocity.append(angel_dif[j]/run_time)
            if msg.velocity[j] < 0.02:
                msg.velocity[j] = 0.02

        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo(msg)
        
        if (simulation == 'false'):
            while not robot_reached(msg.position,js.position):
                pub.publish(msg)
                rate.sleep()

        i = i + 1 
        msg.position=[] 
        msg.velocity=[]
        if (simulation == 'true'):
            rate.sleep()

def shutdown():
    global js
    stop_msg = JointState()
    stop_msg.position = js.position
    for i in range(num_joints):
        stop_msg.velocity[i] = 0.01
    pub.publish(stop_msg)

if __name__ == '__main__':
    try:
        max_vel = math.radians(5)
        if len(sys.argv) < 2 :
           print 'Please input a mrl file path.'
           print 'E.g.: ./mrl_interpretor.py /mrl/***.mrl [max velocity in radians] [simulation]'
        if len(sys.argv)>2:
           max_vel = string.atof(sys.argv[2])
            
           Readfile(sys.argv[1])
           mrl_interpreter(sys.argv[3])

        # Readfile('../mrl/zhizao.mrl')
        rospy.on_shutdown(shutdown)
        shutdown()
    except rospy.ROSInterruptException:
        shutdown()
        pass