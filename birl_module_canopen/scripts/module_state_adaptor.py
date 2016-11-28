#!/usr/bin/env python


#brief:
#      adaptor node between joint_state_publisher and
#      joint_group_position_controller of canopen_motor node
#input:
# 1) encoder resolution and node ID from parameter server
# 2) transmission reduction ratio from URDF,
# 3) /joint_states topic (e.g. topic of joint_state_publisher)
#output:
# 1) publish positions to /joint_group_position_controller/command
# 2) send updated velocity to CAN bus using python-can


from __future__ import print_function
import sys
import rospy
import can
import xml.dom.minidom
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from math import pi
#import pdb


# global
joint_states=JointState()
msg_list={}
node_list={}
bus = can.interface.Bus()
flag=False # first message received flag

def main(arg):
    adapt_node_name = arg
    
    global msg_list,node_list,bus
    
    #load parameters
    try:
        #can_dev = rospy.get_param("/" + adapt_node_name + "/bus/device") #,'can0')
        #default_vel_ratio = rospy.get_param("/" + adapt_node_name + "/defaults/vel_to_device")
        default_resolution = rospy.get_param("/" + adapt_node_name + "/defaults/encoder_resolution")
        CAN_nodes = rospy.get_param("/" + adapt_node_name + "/nodes")
        controlled_joint_names = rospy.get_param("/joint_group_position_controller")['joints']
        description = rospy.get_param("robot_description")
    except KeyError:
        rospy.loginfo("Parameter value not set")
    	
    # get transmission from URDF
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
                continue
        if child.localName == 'transmission':
            try:
                actuator = child.getElementsByTagName('actuator')[0]
                reduction = actuator.childNodes[1]#getElementByTagName('mechanicalReduction')[0]
                reduction_ratio = float(reduction.childNodes[0].nodeValue)
                trans_joint = child.childNodes[3]#getElementByTagName('joint')[0]
                trans_joint_name = trans_joint.getAttribute('name')
            except KeyError:
                rospy.logwarn("No mechanical reduction specified in URDF")
            ratio=reduction_ratio*default_resolution*0.5/pi*10 
            
            for node in CAN_nodes:                
                if  node['name'] == trans_joint_name:
                    node_list[trans_joint_name]={'id':node['id'],'ratio':ratio}

    
    rospy.init_node('module_state_adaptor')
    rospy.Subscriber("/joint_states", JointState, callback)
    
    pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=1)
    r = rospy.Rate(20) # 10hz
    
    #initialize pos
    pos=[]
    for i in range(len(controlled_joint_names)):
        pos.append(0)
        
    rospy.wait_for_message("/joint_states",JointState)
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        i=0
        for jnt in controlled_joint_names:
            pos[i]=msg_list[jnt]['pos']
            i=i+1
        
        pub.publish(Float64MultiArray(data=pos))
        r.sleep()
 

def callback(msg):
    global flag, msg_list, node_list, bus

    #initialize msg_list when first message is received
    if flag == False:
        for i in range(len(msg.name)):
            flag=True
            msg_list[msg.name[i]]={'pos':0,'vel':0}
            
    for i in range(len(msg.name)):
        
        #sending out velocity command using python-can 
        #since canopen_motor doesn't support dynamic object setting
        if msg.velocity[i] != msg_list[msg.name[i]]['vel']:
            id = node_list[msg.name[i]]['id']
            speed = int(node_list[msg.name[i]]['ratio'] * msg.velocity[i])
            can_msg = can.Message(arbitration_id=(0x600+id),
                                  data=[0x23,0x81,0x60,0x00,
                                        speed%256,speed/256%256,
                                        speed/65536%256,
                                        speed/16777216%256],
                                  extended_id=False)
            if bus.send(can_msg) < 0:
                rospy.logwarn("Message NOT sent")
        # update msg_list
        msg_list[msg.name[i]]={'pos':msg.position[i],'vel':msg.velocity[i]}

    
if __name__ == "__main__":
    #pdb.set_trace()
    if len(sys.argv) < 2:
        rospy.logwarn("usage: module_state_adaptor.py <canopen_motor node name>")
    else:
        main(sys.argv[1])
    