#!/usr/bin/env python

"""
linkeway
Jan. 2017
SUBSCRIPTIONS:
  + ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers) ~ pose of all markers detected by ar_track_alvar

"""

import rospy
import copy
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import math
import tf

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

# connect_param has only 4 alternatives due to mechanical pin hole
def descretize_connect_param(angle):
    if math.fabs(angle) < math.pi/4.0:
        return 0
    if math.fabs(angle-math.pi/2) < math.pi/4.0:
        return math.pi/2
    if math.fabs(angle- math.pi) < math.pi/4.0:
        return math.pi
    if math.fabs(angle - math.pi*3/2 ) < math.pi/4.0:
        return math.pi*3/2
    
def find_parent_module(child_marker,child_inversion,child_joint_angle,candidate_parent_markers):
    parent_module_marker = []
    connect_param =0 
    parent_joint_angle =0
    
    if not candidate_parent_markers: # if no unchecked markers
        return [0,0,0,0]

    for candidate_parent_marker in candidate_parent_markers:
        vector= np.array([
                            child_marker.pose.pose.position.x - candidate_parent_marker.pose.pose.position.x,
                            child_marker.pose.pose.position.y - candidate_parent_marker.pose.pose.position.y,
                            child_marker.pose.pose.position.z - candidate_parent_marker.pose.pose.position.z,
                        ])
        vector_mag = np.linalg.norm(vector)
        if vector_mag > Threshold: # if candicate marker too far from child-marker
            continue

        vector_norm= vector / vector_mag
        
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
        candidate_y_axis= np.array([matrix[0][1],matrix[1][1] ,matrix[2][1]])
        candidate_x_axis= np.array([matrix[0][0],matrix[1][0] ,matrix[2][0]])
        candidate_z_axis= np.array([matrix[0][2],matrix[1][2] ,matrix[2][2]])

        if database_dic[child_marker.id] in ['T', 't'] and child_inversion== 'inverted':
            if np.dot(vector_norm, candidate_y_axis) < -math.cos(26.0*math.pi/180.0):
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'inverted'
            elif database_dic[candidate_parent_marker] in ['T','t']:
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'upright'
            elif np.dot( vector_norm, candidate_y_axis) >  math.cos(26.0*math.pi/180.0):
                parent_module_marker= candidate_parent_marker # parent found
                parent_module_inversion= 'inverted'


        else: # for markers of these kind, parent should locate on y_axis
            # if candicate marker not on y_axis of child with err of 土 26.0 degree 
            if np.dot( vector_norm, y_axis) < -math.cos(26.0*math.pi/180.0) and child_inversion == 'inverted':
                # in this case child belongs to non-Tt type inverted
                parent_module_marker= candidate_parent_marker # parent found

                # find out inversion direction
                angle_y = math.acos(np.dot(y_axis,candidate_y_axis))
                if math.fabs(angle_y) <  26.0*math.pi/180.0: # if 2 y-axes are almost the same
                    parent_module_inversion= 'inverted'
                else:
                    parent_module_inversion= 'upright'

                # find out connection and joint_angle
                # Todo

                
                break

            # if candicate marker not on y_axis of child with err of 土 26.0 degree 
            print math.acos(np.dot( vector_norm, y_axis))*180/math.pi
            if np.dot( vector_norm, y_axis) >  math.cos(26.0*math.pi/180.0) and child_inversion == 'upright':
                # in this case child is upright
                parent_module_marker= candidate_parent_marker # parent found

                # find out inversion direction
                angle_y = math.acos(np.dot(y_axis,candidate_y_axis))
                if math.fabs(angle_y - math.pi) <  26.0*math.pi/180.0: #if 2 y-axes are almost opposite
                    parent_module_inversion= 'inverted'
                else:
                    parent_module_inversion= 'upright'

                # find out connection and joint_angle
                if database_dic[parent_module_marker.id] in ['t','T']: 
                    if parent_module_inversion == 'upright':
                        parent_joint_angle= angle_y # TODO angle between 0 to pi
                        connect_param= math.acos(np.dot(z_axis,candidate_z_axis)) # TODO
                    else:
                        a=1
                else:
                    a=1
                break

    if parent_module_marker == []:
        rospy.logerr( 'can\'t find parent marker for marker id: {}'.format(child_marker.id))
        return [0,0,0,0]
    return [parent_module_marker, parent_module_inversion, connect_param, parent_joint_angle] 
# Todo: do it in a recursive way instead of a ugly return
# chain= []
# def find_chain(child_marker, markers):
#   if not markers == True:
#       return
#   ...find out parent_marker...
#   markers.remove(parent_marker)
#   find_chain(parent_marker,markers)
#   chain.append(parent_marker) #chain stores markers from base to end-effector


if __name__ == "__main__":
    rospy.init_node("setup_detector", log_level=rospy.INFO)

    load_module_data("../module_database/modules.dat")

    # receive message from /ar_pose_marker topic
    receive_marker_msg("/ar_pose_marker")
    
    chain= [] # list that stores module type, inversion, assembly parameter from end-effector to base
    for marker in markers:
        if database_dic[marker.id] in ['t','g', 'G', 'S', 'W']: # marker on end-effector
            inversion= 'upright'
            joint_angle= 0
            
            while markers: # while not empty  
                markers.remove(marker)
                [parent_module_marker, parent_module_inversion, connect_param, parent_joint_angle]=find_parent_module(marker,inversion,joint_angle,markers)
                module_state=   { 
                                    'type': database_dic[marker.id],
                                    'inversion': inversion,
                                    'connect_param': connect_param, 
                                    'jonit_angle': joint_angle
                                }
                chain.append(module_state)
              
                marker=copy.deepcopy(parent_module_marker)
                inversion= parent_module_inversion
                joint_angle= parent_joint_angle
            
            break

    chain.reverse() # from base to end-effector
    for node in chain:
        rospy.loginfo( "module_type:{}; direction:{}; connection:{}; joint_angle:{}"
                  .format(node['type'],node['inversion'],node['connect_param'],node['joint_angle']) )





