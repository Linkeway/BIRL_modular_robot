#!/usr/bin/env python

"""
Author: Linkeway
E-mail: linkeway@163.com
Description: end-effector interactive marker node. Publish /marker_pose and calls ik_node services to chang its publish behaviours.
Args:
        base_frame: tf frame name of base
        eef_frame:  tf frame name of end-effector
Publish: /marker_pose
Subscribe: tf
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from geometry_msgs.msg import Pose
import  sys

server = None
menu_handler = MenuHandler()
br = None
counter = 0
pub = None 

base_frame = 'base_link'
eef_frame = 'tcp'

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "marker_frame" )
    counter += 1

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        # rospy.loginfo( s + ": button click" + mp + "." )
        None
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        #rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        if feedback.menu_entry_id == 1: # menu "End-effector On/Off"
            # TODO: once clicked, command the end-effector
            None
        elif feedback.menu_entry_id == 3: # menu "Continuous publishing"
            # TODO: change joint_command pub mode to continuous publishing
            None
        elif feedback.menu_entry_id == 4: # menu "Publish on click"
            # TODO: change joint_command pub mode to publish once when "Publish Joint_command once" clicked
            None
        elif feedback.menu_entry_id == 5: # menu "Publish Joint_command once"
            # TODO: publish joint_command once
            None
        elif feedback.menu_entry_id == 6: # menu "align marker to eef"
            server.setPose( feedback.marker_name, get_eef_pose(base_frame,eef_frame) )
            server.applyChanges()
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        pub.publish(feedback.pose)
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        # rospy.loginfo( s + ": mouse down" + mp + "." )
        None
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        # rospy.loginfo( s + ": mouse up" + mp + "." )
        None
    server.applyChanges()

def alignMarker( feedback ):
    
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

#def rand( min_, max_ ):
#    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)

def make6DofMarker(frame_name, fixed, interaction_mode, pose, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_name
    int_marker.pose = pose
    int_marker.scale = 0.1

    int_marker.name = "eef_marker"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {  InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                                InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                                InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


def makeMenuMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "context_menu"
    int_marker.description = "Context Menu\n(Right Click)"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    # make one control showing a box
    marker = makeBox( int_marker )
    control.markers.append( marker )
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

def makeMovingMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "marker_frame"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "eef_marker"
    int_marker.description = "Marker Attached to a\nMoving Frame"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append( makeBox(int_marker) )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

def get_eef_pose(base_frame,eef_frame):
    '''listen to tf to get relative pose from base_frame to eef_frame'''
    tf_lis = TransformListener()
    tf_lis.waitForTransform(base_frame,eef_frame, rospy.Time(), rospy.Duration(30.0))
    now = rospy.Time.now()
    tf_lis.waitForTransform(base_frame,eef_frame, now, rospy.Duration(4.0))
    (trans,rot) = tf_lis.lookupTransform(base_frame,eef_frame,now )
    p = Pose()
    p.position = Point(trans[0],trans[1],trans[2])
    p.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])
    return p

if __name__=="__main__":

    if len(sys.argv) < 3:
        print "needs an argument : base frame and end-effector frame (default '/base_link' '/tcp')."
    else:
        base_frame = sys.argv[1]
        eef_frame = sys.argv[2]
    
    rospy.init_node("eef_interactive_marker")
    # br = TransformBroadcaster()
    pub = rospy.Publisher("marker_pose",Pose,queue_size=3)
    # create a timer to update the published transforms
#    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("InteractiveMarkerServer")

    # call ik_node services to change its publishing behaviours 
    menu_handler.insert( "End-effector On/Off", callback=processFeedback ) 
    sub_menu_handle = menu_handler.insert( "Joint_command publication mode" ) 
    menu_handler.insert( "Continuous publishing", parent=sub_menu_handle, callback=processFeedback ) 
    menu_handler.insert( "Publish on click", parent=sub_menu_handle, callback=processFeedback ) 
    menu_handler.insert( "Publish Joint_command once", callback=processFeedback ) 

    # when this menu item clicked,the interactive marker get aligned
    menu_handler.insert( "align marker to eef", callback=processFeedback ) 
 
    make6DofMarker(base_frame,False,InteractiveMarkerControl.MOVE_ROTATE_3D,get_eef_pose(base_frame,eef_frame),True )

    server.applyChanges()

    rospy.spin()