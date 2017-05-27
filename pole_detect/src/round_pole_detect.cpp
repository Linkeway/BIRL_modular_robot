// #include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <time.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #define _DEBUG

using  std::string;

typedef pcl::PointXYZ PointT;

// sensor_msgs::PointCloud2 cloud_msg;

// void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//     cloud_msg = *input;
// }

/** \brief Subscribe to topic \b input and detect cylinder
  * \input pointcloud2 topic \b input
  * \return cylinder model coefficients: position + cylinder axis + cylinder radius
  */
pcl::ModelCoefficients::Ptr detect_cylinder()
{
   // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices); 
   
    pcl::fromROSMsg<PointT>( *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("input"),*cloud);

    clock_t start_time = clock();

  // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.0);
    pass.filter (*cloud_filtered);
    if (cloud_filtered->points.size()<=0){
          std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
          throw ("Input pointcloud has 0 points.");
    }
      
  // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
  // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // mean normal of inliers
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_cylinder);
    extract_normals.setNegative (false);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_cylinder_normal (new pcl::PointCloud<pcl::Normal> ());
    extract_normals.filter (*cloud_cylinder_normal);
    pcl::Normal mean_normal;
    tf::Vector3 y_axis_vector;
    for(pcl::PointCloud<pcl::Normal>::iterator it= cloud_cylinder_normal->begin();
                  it !=  cloud_cylinder_normal->end(); ++it)
    {
      y_axis_vector.setX(y_axis_vector.x() + (*it).normal_x);
      y_axis_vector.setY(y_axis_vector.y() + (*it).normal_y);
      y_axis_vector.setZ(y_axis_vector.z() + (*it).normal_z);
    }
    y_axis_vector = y_axis_vector / cloud_cylinder_normal->points.size();
    
  #ifdef _DEBUG
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl ;
    std::cerr <<"Detecting time taken:  " << (double)(clock() - start_time)/CLOCKS_PER_SEC << endl;
  #endif
    return coefficients_cylinder;
}

/** \brief Extract pose information from cylinder model coefficients. \b Note: cylinder coefficients are not enough to determine a pose
  * \param cylinder model coefficients
  * \return cylinder pose
  */
geometry_msgs::PoseStamped getCylinderPose(const pcl::ModelCoefficients::Ptr coefficients_cylinder){
  geometry_msgs::PoseStamped p;
  p.header.frame_id = coefficients_cylinder->header.frame_id;
  pcl_conversions::fromPCL(coefficients_cylinder->header,p.header);

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  p.pose.position.x = coefficients_cylinder->values[0];
  p.pose.position.y = coefficients_cylinder->values[1];
  p.pose.position.z = coefficients_cylinder->values[2];

  tf::Vector3 axis_vector(coefficients_cylinder->values[3], 
                          coefficients_cylinder->values[4], 
                          coefficients_cylinder->values[5]);
  tf::Vector3 up_vector(0.0, 0.0, 1.0);
  tf::Vector3 right_vector = axis_vector.cross(up_vector);
  right_vector.normalized();
  tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
  q.normalize();
  geometry_msgs::Quaternion cylinder_orientation;
  tf::quaternionTFToMsg(q, cylinder_orientation);
  p.pose.orientation = cylinder_orientation;
  
  return p;
}

inline float getCylinderRadius(const pcl::ModelCoefficients::Ptr coefficients_cylinder){
  return coefficients_cylinder->values[6];
}

/** \brief Publish cylinder marker messages for visualization in Rviz
  * \param cylinder pose
  * \param cylinder radius
  * \param marker publisher
  * \output marker topic 
  */
void publish_cylinder_marker(geometry_msgs::PoseStamped& p,float radius,ros::Publisher& marker_pub)
{
      visualization_msgs::Marker marker;

      marker.header.frame_id = p.header.frame_id;
      
      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "detected_cylinder_pole";
      marker.id = 0;

      // Set the marker type.  CUBE, SPHERE, ARROW, CYLINDER etc.
      marker.type = visualization_msgs::Marker::CYLINDER;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose = p.pose;
    
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 2*radius;
      marker.scale.y = 2*radius;
      marker.scale.z = 1.0;

      // Set the color 
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      // while (marker_pub.getNumSubscribers() < 1)
      // {
      //   if (!ros::ok()){
      //     return ;
      //   }
      //   ROS_WARN_ONCE("Please create a subscriber to the marker");
      //   sleep(1);
      // }
      marker.header.stamp = ros::Time::now();
      marker_pub.publish(marker);
}

void broadcastCylinderTF(geometry_msgs::PoseStamped& ps,tf::TransformBroadcaster& br){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z));
  transform.setRotation( tf::Quaternion(ps.pose.orientation.x,ps.pose.orientation.y,
                                        ps.pose.orientation.z,ps.pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ps.header.stamp, ps.header.frame_id, "detected_pole"));
}

geometry_msgs::Pose generateCylinderGraspPose(geometry_msgs::PoseStamped& cylinder_pose){
  

}

void myTransformStampedTFToMsg(tf::StampedTransform& st,geometry_msgs::PoseStamped & ps){
  ps.header.stamp = st.stamp_;
  // ps.header.frame_id = st.frame_id_;
  ps.pose.position.x = st.getOrigin()[0];
  ps.pose.position.y = st.getOrigin()[1];
  ps.pose.position.z = st.getOrigin()[2];
  ps.pose.orientation.x = st.getRotation().getX();
  ps.pose.orientation.y = st.getRotation().getY();
  ps.pose.orientation.z = st.getRotation().getZ();
  ps.pose.orientation.w = st.getRotation().getW();
}

int main(int argc, char** argv){
  ros::init (argc, argv, "round_pole_detect");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  tf::TransformListener tf_lis;
  tf::TransformBroadcaster br;

  // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("detected_pole", 1);
  ros::Publisher pole_pose_pub = nh.advertise<geometry_msgs::Pose>("pole_pose",1);





  while (ros::ok()){
    tf::StampedTransform transform;
    pcl::ModelCoefficients::Ptr coefficients_cylinder;  
    geometry_msgs::PoseStamped cylinder_pose;

    //detect round pole
    try{
      coefficients_cylinder = detect_cylinder();
      cylinder_pose = getCylinderPose(coefficients_cylinder); //cylinder_pose relative to camera frame
    }catch(...){
      continue;
    }

    broadcastCylinderTF(cylinder_pose,br);
    // transform from base_frame to detected_pole
    try{
      tf_lis.lookupTransform("detected_pole","base_gripper1",cylinder_pose.header.stamp,transform);
      myTransformStampedTFToMsg(transform, cylinder_pose); //cylinder_pose relative to base_frame
      pole_pose_pub.publish(cylinder_pose.pose);
    }catch(...){
      
    }
    
    // publish_cylinder_marker(cylinder_pose,getCylinderRadius(coefficients_cylinder),marker_pub);
    
    ros::spinOnce();
  }
    
    return 0;
}