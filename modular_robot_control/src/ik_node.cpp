/* *******************************************************************************
TODO: Add services to support its publishing behaviours.
********************************************************************************/

//#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <kdl_conversions/kdl_msg.h>
#include <vector>
#include <math.h>
 

geometry_msgs::Pose pose_command;
sensor_msgs::JointState robot_joint_states;
bool new_pose = true;

void pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
    new_pose = true;
    pose_command=*msg;
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  robot_joint_states=*msg;
}

double normalize_angel(double angel){
  while (angel < -M_PI)
    angel += 2*M_PI;
  while (angel > M_PI)
    angel -= 2*M_PI;
  return angel;
}
int main(int argc, char** argv)
{

  bool sim = true;
  if (strcmp(argv[1], "false") == 0)
    sim = false;
  ros::init(argc, argv, "ik_node");
  ros::NodeHandle nh("~");

  std::string chain_start, chain_end, urdf_param;
  std::vector<std::string> joint_names;
  double timeout;

  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));

  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain_start or chain_end parameter info in launch file.");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  double eps = 0.02;

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
  
  while(!nh.hasParam("/joint_names")){
    ROS_INFO("Waiting for parameter '/joint_names'");
    sleep(1);
  }
  ROS_INFO("Get parameter '/joint_names'");
  nh.getParam("/joint_names",joint_names);

  sensor_msgs::JointState msg;
  msg.name = joint_names;
  msg.position.resize(joint_names.size());
  msg.velocity.resize(joint_names.size());
  robot_joint_states.position.resize(joint_names.size());
  robot_joint_states.velocity.resize(joint_names.size());
  // for(int i=0;i<joint_names.size();++i)
  //   msg.velocity[i] = 0.04;
  ros::Rate pub_rate(10);
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_command",2);
  ros::Subscriber sub = nh.subscribe("/marker_pose", 2, pose_callback);
  ros::topic::waitForMessage<geometry_msgs::Pose>("/marker_pose");
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states",2,joint_state_callback);
  
  std::vector<double> angel_diff;
  angel_diff.resize(joint_names.size());
  double max_angel_diff;
  double max_vel=0.08;
  double run_time;

  int rc;
  while(ros::ok()){
    tf::poseMsgToKDL(pose_command,end_effector_pose);
    rc = tracik_solver.CartToJnt(result,end_effector_pose,result);
    if (rc>=0){//ik success
       ROS_INFO_STREAM("*** TRAC-IK successed. rc: "<<rc<<". Result: "); 
       for(uint j=0; j<result.data.size(); j++) 
       {
          ROS_INFO_STREAM(" "<<result(j));
          //ROS_INFO_STREAM("ik solution found.");
          msg.position[j] = result(j);
          if (sim == false)
          {
            angel_diff[j] = abs(msg.position[j]-robot_joint_states.position[j]);
          }     
       }
       if (sim == false){
          max_angel_diff = *max_element(angel_diff.begin(),angel_diff.end());
          run_time = max_angel_diff / max_vel;
          if (run_time < 0.01)
            run_time = 0.01;
          for(uint j=0; j<result.data.size(); j++){
            msg.velocity[j] = angel_diff[j]/run_time;
            if (msg.velocity[j]<0.02)
              msg.velocity[j]=0.02; 
          }
       }
       msg.header.stamp = ros::Time::now();
       new_pose = false;
       while(!new_pose){ //wait for new pose
         pub.publish(msg);
         ros::spinOnce();
         pub_rate.sleep();
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
