#!/usr/bin/env python

"""
linkeway
Jan. 2017

PARAMETERS:
  + /robot_name ~ default: 'robot'
SUBSCRIPTIONS:
  + ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers) ~ pose of all markers detected by ar_track_alvar
OUTPUT:
Automatically generate robot.urdf.xacro to [PATH_TO_MR_DESCRIPTION_PACKAGE]/robot/robot.urdf.xacro
Automatically generate robot_display.launch to [PATH_TO_MR_DESCRIPTION_PACKAGE]/lauch/robot_display.launch

"""

import rospy
import copy
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import math
import tf
import rospkg
import sys 
import roslaunch
from sensor_msgs.msg import JointState

Threshold= 0.26
database_dic={} 
markers=[]

# load from a database that keeps all modules information to database_dic
def load_module_data(file_name):
    database = open(file_name, "r")
    for line in database:
        [tagID,type] = line.strip('\n').split(': ',1)
        database_dic[int(tagID)]=type
    database.close()

# topic Subscriber
def receive_marker_msg(topic):
    detected_tag_id=[]
    msg = rospy.wait_for_message(topic, AlvarMarkers)
    for marker in msg.markers:
        id = marker.id
        if id not in detected_tag_id:
            if id in database_dic.keys():
                detected_tag_id.append(id)
                markers.append( copy.deepcopy(marker) )

# connect_angle has only 4 alternatives due to mechanical pin hole
def descretize_connect_angle(angle):
    if math.fabs(angle) < math.pi/4.0:
        return 0
    if math.fabs(angle-math.pi/2) < math.pi/4.0:
        return math.pi/2
    if math.fabs(angle- math.pi) < math.pi/4.0:
        return math.pi
    if math.fabs(angle - math.pi*3/2 ) < math.pi/4.0:
        return math.pi*3/2
    
# INPUT: child_marker and its state, unchecked markers
# OUTPUT: state of parent module_state
# TODO: use combinatorial optimization methods instead of hardcoded geometrical constraints to achevie generality of the solution
def find_parent_module(child_marker,child_inversion,child_joint_angle,candidate_parent_markers):
    parent_module_marker = []
    connect_angle =0 
    parent_joint_angle =0
    
    if not candidate_parent_markers: # if no unchecked markers
        return [0,0,0,0]

    # find parent_module and its inversion
    for candidate_parent_marker in candidate_parent_markers:
        vector= np.array([
                            child_marker.pose.pose.position.x - candidate_parent_marker.pose.pose.position.x,
                            child_marker.pose.pose.position.y - candidate_parent_marker.pose.pose.position.y,
                            child_marker.pose.pose.position.z - candidate_parent_marker.pose.pose.position.z,
                        ])
        vector_mag = np.linalg.norm(vector)
        if vector_mag > Threshold: # if candicate marker too far from child-marker
            continue

        unit_vector= vector / vector_mag
        
        matrix= tf.transformations.quaternion_matrix(  [child_marker.pose.pose.orientation.x,
                                                        child_marker.pose.pose.orientation.y,
                                                        child_marker.pose.pose.orientation.z,
                                                        child_marker.pose.pose.orientation.w])
        y_axis= np.array([matrix[0][1],matrix[1][1] ,matrix[2][1]])
        x_axis= np.array([matrix[0][0],matrix[1][0] ,matrix[2][0]])
        z_axis= np.array([matrix[0][2],matrix[1][2] ,matrix[2][2]])
        matrix= tf.transformations.quaternion_matrix([candidate_parent_marker.pose.pose.orientation.x,
                                                     candidate_parent_marker.pose.pose.orientation.y,
                                                     candidate_parent_marker.pose.pose.orientation.z,
                                                     candidate_parent_marker.pose.pose.orientation.w])
        candidate_y_axis= np.array([matrix[0][1], matrix[1][1], matrix[2][1]])
        candidate_x_axis= np.array([matrix[0][0], matrix[1][0], matrix[2][0]])
        candidate_z_axis= np.array([matrix[0][2], matrix[1][2], matrix[2][2]])

        if database_dic[child_marker.id] in ['T', 't'] and child_inversion== 'inverted':
            if np.dot(unit_vector, candidate_y_axis) < -math.cos(26.0*math.pi/180.0):
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'inverted'
            elif database_dic[candidate_parent_marker.id] in ['T','t']:
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'upright'
            elif np.dot( unit_vector, candidate_y_axis) >  math.cos(26.0*math.pi/180.0):
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'upright'

        else: # for markers of these kind, parent should locate on y_axis
            # if candicate marker not on y_axis of child with err of 26.0 degree 
            if np.dot( unit_vector, y_axis) < -math.cos(26.0*math.pi/180.0) and child_inversion == 'inverted':
                # in this case child belongs to non-Tt type inverted
                parent_module_marker= candidate_parent_marker # parent found

                # find out inversion direction
                angle_y = math.acos(np.dot(y_axis,candidate_y_axis))
                if math.fabs(angle_y) <  26.0*math.pi/180.0: # if 2 y-axes are almost the same
                    parent_module_inversion= 'inverted'
                else:
                    parent_module_inversion= 'upright'
                break

            # if candicate marker not on y_axis of child with err of 26.0 degree 
            # print math.acos(np.dot( unit_vector, y_axis))*180/math.pi
            if np.dot( unit_vector, y_axis) >  math.cos(26.0*math.pi/180.0) and child_inversion == 'upright':
                # in this case child is upright
                parent_module_marker= candidate_parent_marker # parent found

                # find out inversion direction
                angle_y = math.acos(np.dot(y_axis,candidate_y_axis))
                if math.fabs(angle_y - math.pi) <  26.0*math.pi/180.0: #if 2 y-axes are almost opposite
                    parent_module_inversion= 'inverted'
                else:
                    parent_module_inversion= 'upright'

    if parent_module_marker == []:
        rospy.logerr( 'can\'t find parent marker for marker id: {}'.format(child_marker.id))
        return [0,0,0,0]

    # calculate connect_angle, which is the angle between two z axes of markers
    z_axes_angle = math.acos( np.clip( np.dot(z_axis,candidate_z_axis), -1, 1))
    if np.dot(unit_vector, np.cross(z_axis, candidate_z_axis) ) > 0: # use unit_vector to determine positive direction of connect_angle 
        connect_angle = descretize_connect_angle( z_axes_angle )
    else:
        connect_angle = descretize_connect_angle( 2*math.pi - z_axes_angle )
    
    # find out parent_joint_angle
    if database_dic[parent_module_marker.id] in ['t','T']:
        if parent_module_inversion == 'upright':
            parent_joint_angle = math.acos( np.clip( np.dot(unit_vector,candidate_y_axis), -1, 1))
            if np.dot(candidate_z_axis, np.cross(unit_vector, candidate_y_axis) ) < 0: # use z_axis to determine positive direction of angle 
                parent_joint_angle = - parent_joint_angle
        else:
            # TODO: find out joint_angle for T/t inverted module 
            None

    return [parent_module_marker, parent_module_inversion, connect_angle, parent_joint_angle] 


# writes xacros to urdf_file_path according to template file specified by template_file_path 
def create_urdf_file(chain_list,urdf_file_path,template_file_path):
    with open(urdf_file_path, "w") as urdf_file:

        with open(template_file_path) as template_file:
            for line in template_file:
                urdf_file.write(line)
            template_file.close()
        cnt=1
        urdf_file.write("  <link name=\"base_link\"/>\n\n")

        for module in chain_list:
            
            if module['inversion'] == 'upright':
                inversion = 'module'
            else:
                inversion = 'invert'

            if cnt == 1:
                parent_link = 'base_link'
            
            type = module['type']
            urdf_file.write("  <xacro:{}_{} name=\"{}{}\" parent=\"{}\">\n".format(type,inversion,type,cnt,parent_link) )
            urdf_file.write("    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n")
            urdf_file.write("  </xacro:{}_{}>\n\n".format(type,inversion))

            if parent_link[0] in ['G','I','T'] and type in ['g','i','t']:
                urdf_file.write("  <xacro:connect_link_100_85  name=\"cl{}\" parent=\"{}\">\n".format( "{}-{}".format(cnt,cnt+1), parent_link) )
                urdf_file.write("    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n")
                urdf_file.write("  </xacro:connect_link_100_85>\n\n")

            if parent_link[0] in ['g','i','t'] and type in ['G','I','T']:
                urdf_file.write("  <xacro:connect_link_85_100  name=\"cl{}\" parent=\"{}\">\n".format( "{}-{}".format(cnt,cnt+1), parent_link) )
                urdf_file.write("    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n")
                urdf_file.write("  </xacro:connect_link_85_100>\n\n")

            parent_link = '{}{}_Link'.format(type,cnt)
            cnt = cnt +1

        urdf_file.write("\n</robot>")
        urdf_file.close()

# create a .launch file and a .rviz(optionally) in order to display or visualize xacro_file
def create_launch_file(launch_file, xacro_file, chain, rviz_conf_file = []):
    with open(launch_file, "w") as file:
        file.write('<launch>\n\n')
        file.write('  <arg name="gui" default="true" />\n\n')
        file.write('  <param name=\"robot_description\" command=\"$(find xacro)/xacro \'{}\' \"/>\n'.format(xacro_file))
        file.write('  <node name=\"module_state_publisher\" pkg=\"joint_state_publisher\" type=\"joint_state_publisher\">\n')
        file.write('    <param name=\"use_gui\" value=\"true\"/>\n')
        file.write('  </node>\n')
        file.write('  <node name=\"robot_state_publisher\" pkg=\"robot_state_publisher\" type=\"state_publisher\" />\n')
        if rviz_conf_file == []:
            file.write('  <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-f base_link\" if=\"$(arg gui)\"/>\n\n')
        else:
            file.write('  <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d {} -f base_link\" if=\"$(arg gui)\"/>\n\n'.format(rviz_conf_file))
        
        # write dependent_joints parameters of gripper module
        cnt=1
        for module in chain:
            if module['type'] in ['g', 'G']:
                module_name = '{}{}'.format(module['type'],cnt)
                file.write('  <rosparam>\n')
                file.write('    dependent_joints:\n')
                file.write('      {0}_Joint1: {1} parent: {0}_Joint, factor: 1 {2}\n'.format(module_name, '{', '}') )
                file.write('  </rosparam>\n\n')
            cnt = cnt+1

        file.write('</launch>')
    file.close()

if __name__ == "__main__":
    rospy.init_node("setup_detector", log_level=rospy.INFO)

    robot_name = "robot"
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    
    load_module_data("../module_database/modules.dat")

    # receive message from /ar_pose_marker topic
    receive_marker_msg("/ar_pose_marker")
    
    chain= [] # list that stores module type, inversion, assembly parameter from end-effector to base
    for marker in markers:
        if database_dic[marker.id] in ['g', 'G', 'S', 'W']: # if is marker on end-effector
            inversion= 'upright'
            joint_angle= 0
            
            while markers: # while not empty  
                markers.remove(marker)
                [parent_module_marker, parent_module_inversion, connect_angle, parent_joint_angle]=find_parent_module(marker,inversion,joint_angle,markers)
                module_state=   { 
                                    'type': database_dic[marker.id],
                                    'inversion': inversion,
                                    'connect_angle': connect_angle, 
                                    'joint_angle': joint_angle
                                }
                chain.append(module_state)
              
                marker=copy.deepcopy(parent_module_marker)
                inversion= parent_module_inversion
                joint_angle= parent_joint_angle
            break
    chain.reverse() # from base to end-effector

            # TODO: do it in a recursive way instead of a ugly return in find_parent_module()
            # chain= []
            # def find_chain(child_marker, unchecked_markers):
            #   if not unchecked_markers:
            #       return
            #   ...find out parent_marker...
            #   unchecked_markers.remove(parent_marker)
            #   find_chain(parent_marker, unchecked_markers)
            #   chain.append(parent_marker) #chain stores markers from base to end-effector
            
    for node in chain:
        rospy.loginfo( "module_type:{}; direction:{}; connection:{}; joint_angle:{}"
                  .format(node['type'],node['inversion'],node['connect_angle'],node['joint_angle']) )

    package_path = rospkg.RosPack().get_path('mr_description')
    xacro_file = package_path + '/robots/{}.urdf.xacro'.format(robot_name)
    template_file = package_path + '/robots/mr_xacro_template.urdf.xacro'
    create_urdf_file(chain, xacro_file, template_file)

    launch_file = package_path + '/launch/{}_display.launch'.format(robot_name)
    rviz_conf_file = package_path + '/rviz/{}.rviz'.format(robot_name)
    create_launch_file(launch_file, xacro_file, chain, rviz_conf_file)
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])  
    launch.start()
    
    #TODO: publish joint states
    rospy.sleep(10)
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    cnt = 1
    for node in chain:
        joint_name = "{}{}_Joint".format(node['type'], cnt)
        cnt = cnt +1
        msg.name.append(joint_name)
        msg.position = node['joint_angle']

    pub = rospy.Publisher('joint_states', JointState, queue_size=5)
    pub.publish(msg)

    while not rospy.is_shutdown():
        None
    launch.shutdown()