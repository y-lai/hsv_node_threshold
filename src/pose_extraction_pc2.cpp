#include "hsv_node_threshold/pose_extraction_pc2.h"

pose_extraction_pc2::pose_extraction_pc2(ros::NodeHandle pnh,ros::NodeHandle nh)
{
  _pnh = new ros::NodeHandle(pnh);
  if(!setup(nh))
  {
    ROS_ERROR("Unable to find parameters. Please check roslaunch file!");
    return;
  }
  _point_sub = nh.subscribe(_point_sub_name.c_str(),1,&pose_extraction_pc2::pointcloudCB,this);
  _pose_array_pub = _pnh->advertise<geometry_msgs::PoseArray>(_pose_array_pub_name.c_str(),1,true);
  _cluster_points_pub = _pnh->advertise<sensor_msgs::PointCloud2>(_cluster_points_pub_name.c_str(),1,true);

  _ec.setClusterTolerance(_cluster_tolerance);
  _ec.setMinClusterSize(_min_size);
  _ec.setMaxClusterSize(_max_size);

  try{
    _listen.waitForTransform("/kinect2_link","/base",ros::Time(0),ros::Duration(5.0));
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }
  _listen.lookupTransform("/base","/kinect2_link",ros::Time(0),_stamped_trans);
  _trans.setOrigin(_stamped_trans.getOrigin());
  _trans.setRotation(_stamped_trans.getRotation());
  tf::transformTFToEigen(_trans,_eigen_tf);

  while(ros::ok() && _pnh->ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("Ending class pose_extraction_pc2 constructor");
}

bool pose_extraction_pc2::setup(ros::NodeHandle nh)
{
  bool temp = true;

  if(!nh.getParam("points_pub_name",_point_sub_name))
  {
    temp = false;
    ROS_ERROR("Unable to find point_sub_name parameter in global param server");
  }

  if(!nh.getParam("hsv_name",_hsv_node_name))
  {
    temp = false;
    ROS_ERROR("Unable to find hsv_name parameter in global param server");
  }

  if(!_pnh->getParam("pose_array_pub_name",_pose_array_pub_name))
  {
    temp = false;
    ROS_INFO("Unable to find pose_array_pub_name parameter");
  }

  if(!_pnh->getParam("cluster_points_pub_name",_cluster_points_pub_name))
  {
    temp = false;
    ROS_INFO("Unable to find cluster_points_pub_name parameter");
  }

  std::vector<double> tempvec;
  if(!_pnh->getParam("ec_params",tempvec))
  {
    temp = false;
    ROS_INFO("Unable to find ec_params parameter");
  }
  else
  {
    _cluster_tolerance = tempvec[0];
    _min_size = tempvec[1];
    _max_size = tempvec[2];
    ROS_ERROR("Current cluster tolerance from parameter server: %f",_cluster_tolerance);
    ROS_ERROR("Current min size: %f    Current Max Size: %f",_min_size,_max_size);
  }


  return temp;
}

bool pose_extraction_pc2::extract(sensor_msgs::PointCloud2 input)
{
  pcl::PointCloud<pcl::PointXYZRGB> temp_pc2;
  pcl::fromROSMsg(input,temp_pc2);
  std::vector<int> temp_index;
  ROS_INFO("Size of input: %lu",temp_pc2.points.size());
  temp_pc2.is_dense = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _latest(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::removeNaNFromPointCloud(temp_pc2,*_latest,temp_index);
  ROS_INFO("Size of output: %lu",_latest->points.size());

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(_latest);
  _ec.setSearchMethod(tree);
  _ec.setInputCloud(_latest);
  _cluster_indices.clear();
  _ec.extract(_cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB> _out_temp;
  geometry_msgs::PoseArray temp_pose_array;
  for(std::vector<pcl::PointIndices>::iterator it = _cluster_indices.begin();it!=_cluster_indices.end();it++)
  {
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    for(std::vector<int>::iterator itt = it->indices.begin();itt != it->indices.end();itt++)
    {
      _out_temp.points.push_back(_latest->points[*itt]);
      temp.points.push_back(_latest->points[*itt]);
    }
    pcl::PointCloud<pcl::PointXYZRGB> temp_transform;
    pcl::transformPointCloud(temp,temp_transform,_eigen_tf);
    geometry_msgs::Pose temp_pose = computePose(temp_transform);
    if(temp_pose.position.x > 0.0 && temp_pose.position.z > 0.0)
    {
      temp_pose_array.poses.push_back(computePose(temp));
    }
  }
  _output = temp_pose_array;

  pcl::toROSMsg(_out_temp,_cluster_output);

  if(_output.poses.size() > 6)
  {
    return false;
  }

  return true;
}

geometry_msgs::Pose pose_extraction_pc2::computePose(pcl::PointCloud<pcl::PointXYZRGB> input)
{
  Eigen::Vector4f cent;
  pcl::compute3DCentroid(input,cent);
  geometry_msgs::Pose temp;

  temp.position.x = cent[0];
  temp.position.y = cent[1];
  temp.position.z = cent[2];

  temp.orientation.w = 1.0;
  temp.orientation.x = 0.0;
  temp.orientation.y = 0.0;
  temp.orientation.z = 0.0;

  return temp;
}

void pose_extraction_pc2::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &kin)
{
  if(extract(*kin))
  {
    _cluster_output.header.stamp = ros::Time::now();
    _output.header.stamp = ros::Time::now();
    _cluster_output.header.frame_id = "/kinect2_link";
    _output.header.frame_id = "/base";
    _cluster_points_pub.publish(_cluster_output);
    _pose_array_pub.publish(_output);
  }
}
