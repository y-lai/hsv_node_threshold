#include "hsv_node_threshold/hsv_check.h"

hsv_check::hsv_check(ros::NodeHandle pnh)
{
  _pnh = new ros::NodeHandle(pnh);
  if(!setup())
  {
    ROS_ERROR("Missing parameter argument. Check launch file!");
    return;
  }

  _kin_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*_pnh,_kin_sub_topic.c_str(),1);
  _rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*_pnh,_rgb_sub_topic.c_str(),1);
  _sync = new message_filters::Synchronizer<_pol>(_pol(10),*_rgb_sub,*_kin_sub);

  _kin_pub = _pnh->advertise<sensor_msgs::PointCloud2>(_kin_pub_topic.c_str(),1);
  _sync->registerCallback(boost::bind(&hsv_check::callback,this,_1,_2));

  ros::spin();
}

bool hsv_check::setup()
{
  bool temp = true;
  if(!_pnh->getParam("rgb_sub_topic",_rgb_sub_topic))
  {
    ROS_ERROR("Rgb sub topic not found in parameter server");
    temp = false;
  }

  if(!_pnh->getParam("kin_sub_topic",_kin_sub_topic))
  {
    ROS_ERROR("Kin sub topic not found in parameter server");
    temp = false;
  }

  if(!_pnh->getParam("kin_pub_topic",_kin_pub_topic))
  {
    ROS_ERROR("Kin pub topic not found in parameter server");
    temp = false;
  }

  if(!_pnh->getParam("kin_pub_frame",_kin_pub_frame))
  {
    ROS_ERROR("Kin pub frame not found in parameter server");
    temp =false;
  }

  std::vector<double> vec_temp;
  if(!_pnh->getParam("hsv_lower",vec_temp))
  {
    ROS_ERROR("Hsv lower param not found in parameter server");
    temp = false;
  }
  else
  {
    _hsv_lvec = new cv::Scalar(vec_temp[0],vec_temp[1],vec_temp[2]);
  }

  if(!_pnh->getParam("hsv_upper",vec_temp))
  {
    ROS_ERROR("Hsv upper param not found in parameter server");
    temp = false;
  }
  else
  {
    _hsv_uvec = new cv::Scalar(vec_temp[0],vec_temp[1],vec_temp[2]);
  }
  return temp;
}

void hsv_check::callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::PointCloud2ConstPtr &kin)
{
  pcl::fromROSMsg(*kin,_latest._pcl);
  {
    cv_bridge::CvImageConstPtr out = cv_bridge::toCvShare(rgb,sensor_msgs::image_encodings::BGR8);
    _latest._mat = out->image;
  }
  cv::cvtColor(_latest._mat,_latest._mat,CV_BGR2HSV);
  cv::Mat mask;
  cv::inRange(_latest._mat,*_hsv_lvec,*_hsv_uvec,mask);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_temp;
  for(int i=0;i<mask.rows;i++)
  {
    uchar* M = mask.ptr<uchar>(i);
    for(int j=0;i<mask.cols;j++)
    {
      if(M[j] > 0)
      {
        pcl_temp.points.push_back(_latest._pcl.points[(i*mask.rows)+j]);
      }
    }
  }

  pcl::toROSMsg(pcl_temp,_publish);
  _publish.header.frame_id = _kin_pub_frame.c_str();
  _kin_pub.publish(_publish);
}
