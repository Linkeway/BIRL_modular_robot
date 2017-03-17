/* *******************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

//#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <kdl_conversions/kdl_msg.h>

geometry_msgs::Pose pose_command;

void pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
    pose_command=*msg;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "ik_node");
  ros::NodeHandle nh("~");

  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));

  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain_start or chain_end parameter info in launch file.");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  double eps = 1;

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits


  if (!tracik_solver.getKDLChain(chain)) {
    ROS_ERROR("There was no valid KDL chain found");
    return -1;
  }
  if (!tracik_solver.getKDLLimits(ll,ul)) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return -1;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  // Set up KDL IK and I  K
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  //KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  //KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver

  KDL::JntArray result(chain.getNrOfJoints());
  KDL::Frame end_effector_pose;
  

  // Initialize result with nominal configuration(midway between all joint limits)
  for (uint j=0; j<result.data.size(); j++) {
    result(j) = (ll(j)+ul(j))/2.0;
  }
  //fk_solver.JntToCart(result,end_effector_pose);
  sensor_msgs::JointState msg;


  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states",2);
  ros::Subscriber sub = nh.subscribe("/marker_pose", 2, pose_callback);
  ros::topic::waitForMessage<geometry_msgs::Pose>("/marker_pose");

  int rc;
  while(ros::ok()){
    tf::poseMsgToKDL(pose_command,end_effector_pose);
    rc = tracik_solver.CartToJnt(result,end_effector_pose,result);//end_effector_pose替换成transform listen获取
    if (rc>=0){//ik success
       ROS_INFO_STREAM("*** TRAC-IK successed. rc: "<<rc<<". Result: "); //sevice 替代
       for(uint j=0; j<result.data.size(); j++) {
          ROS_INFO_STREAM(" "<<result(j));
          msg.points.append(result(j));
        pub.publish(msg);
       } 

    }     
    ros::spinOnce();   
  }
  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}
