#!/usr/bin/env python
# see http://gazebosim.org/tutorials?tut=drcsim_animate_joints&cat=

import roslib; #roslib.load_manifest('joint_animation_tutorial')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('traj_publisher_for_manipulator5d')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0: #wait till ros time initializes
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    pub = rospy.Publisher('/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    #jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "base_link"

    jt.joint_names.append("I1_Joint" )
    jt.joint_names.append("T2_Joint" )
    jt.joint_names.append("T3_Joint" )
    jt.joint_names.append("i4_Joint" )
    jt.joint_names.append("t5_Joint" )

    rate = rospy.Rate(10) # 10hz

    n = 200
    dt = 0.2
    rps = 0.1 #indicates period, not the joint velocity
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.4*math.sin(2*theta)
        x2 = 0.4*math.sin(1*theta)

        p.positions.append(0)
        p.positions.append(0)
        p.positions.append(0)
        p.positions.append(x2)
        p.positions.append(x1)

        p.velocities.append(0.05)
        p.velocities.append(0.05)
        p.velocities.append(0.05)
        p.velocities.append(0.09)
        p.velocities.append(0.09)

        jt.points.append(p)

        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt*i)

        rospy.loginfo("Angles[%d]: [%f, %f]",i,x1,x2)

    print "Waiting for the subscriber."
    while pub.get_num_connections() == 0:
        rate.sleep()

    jt.header.stamp = rospy.Time.now()+rospy.Duration(0.05)
    pub.publish(jt)
    print "Message published."
#    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
