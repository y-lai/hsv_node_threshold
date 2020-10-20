#ifndef POSE_EXTRACTION_PC2_H
#define POSE_EXTRACTION_PC2_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
//#include <intera_core_msgs/EndpointState.h>

class pose_extraction_pc2
{
public:
  pose_extraction_pc2(ros::NodeHandle pnh,ros::NodeHandle nh);

private:
  ros::NodeHandle* _pnh;
  ros::Subscriber _point_sub;
  ros::Publisher _pose_array_pub,_cluster_points_pub;

  tf::TransformListener _listen;
  tf::Transform _trans;
  tf::StampedTransform _stamped_trans;
  Eigen::Affine3d _eigen_tf;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> _ec;

  std::vector<pcl::PointIndices> _cluster_indices;
  sensor_msgs::PointCloud2 _cluster_output;
  geometry_msgs::PoseArray _output;

  std::string _point_sub_name,_pose_array_pub_name,_cluster_points_pub_name,_hsv_node_name;
  double _cluster_tolerance,_min_size,_max_size;

  bool setup(ros::NodeHandle nh);
  bool extract(sensor_msgs::PointCloud2 input);
  geometry_msgs::Pose computePose(pcl::PointCloud<pcl::PointXYZRGB> input);
  void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &kin);
};

#endif // POSE_EXTRACTION_PC2_H
