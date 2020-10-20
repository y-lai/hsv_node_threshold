#include "hsv_node_threshold/hsv_node_threshold.h"

hsv_node_threshold::hsv_node_threshold(ros::NodeHandle pnh,ros::NodeHandle nh)
{
  _pnh = new ros::NodeHandle(pnh);
  if(!setup(nh))
  {
    ROS_ERROR("Unable to find parameters. Please check roslaunch file!");
    ROS_INFO("Ending node");
    return;
  }
  _kin_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*_pnh,_kin_sub_name.c_str(),1);
  _rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*_pnh,_rgb_sub_name.c_str(),1);
  _kin_pub = nh.advertise<sensor_msgs::PointCloud2>(_points_pub_name,1);
  _loop = new ros::Rate(_pub_rate);

  _sync = new message_filters::Synchronizer<syncpol>(syncpol(15), *_kin_sub, *_rgb_sub);
  _sync->registerCallback(boost::bind(&hsv_node_threshold::filteredCB,this,_1,_2));


  pub_thread = new std::thread(&hsv_node_threshold::publish,this);

  ros::spin();

  pub_thread->join();

  ROS_INFO("Ending constructor");
}

void hsv_node_threshold::filteredCB(const sensor_msgs::PointCloud2ConstPtr &kin,const sensor_msgs::ImageConstPtr &rgb)
{
  mat_pcl_container temp;
  pcl::fromROSMsg(*kin,temp._pcl);
  temp._mat = (cv_bridge::toCvCopy(*rgb,sensor_msgs::image_encodings::BGR8))->image;
  std::unique_lock<std::mutex> lock(_latest._mtx);
  _latest._buffer.push(temp);
}

bool hsv_node_threshold::setup(ros::NodeHandle nh)  //setup hsv thresholds,pubrate,rgbsub,kinsub,runrate,pointpub
{
  bool temp=true;

  if(!_pnh->getParam("rgb_sub_name",_rgb_sub_name))
  {
    temp = false;
    ROS_INFO("Unable to find rgb_sub_name parameter");
  }

  if(!_pnh->getParam("kin_sub_name",_kin_sub_name))
  {
    temp = false;
    ROS_INFO("Unable to find kin_sub_name parameter");
  }

  if(!nh.getParam("points_pub_name",_points_pub_name))
  {
    temp = false;
    ROS_ERROR("Unable to find points_pub_name parameter in global param server");
  }

  if(!nh.getParam("points_pub_frame_id",_points_pub_frame_id))
  {
    temp = false;
    ROS_ERROR("Unable to find points_pub_frame_id parameter in global param server");
  }

  if(!_pnh->getParam("pub_rate",_pub_rate))
  {
    temp = false;
    ROS_INFO("Unable to find pub_rate parameter");
  }

  std::vector<double> tempvec;
  if(!_pnh->getParam("hsv_lower",tempvec))
  {
    temp = false;
    ROS_INFO("Unable to find hsv_lower parameter");
  }
  else
  {
    lowthres = cv::Scalar(tempvec[0],tempvec[1],tempvec[2]);
  }

  if(!_pnh->getParam("hsv_upper",tempvec))
  {
    temp = false;
    ROS_INFO("Unable to find hsv_upper parameter");
  }
  else
  {
    highthres = cv::Scalar(tempvec[0],tempvec[1],tempvec[2]);
  }

  return temp;
}

bool hsv_node_threshold::run()
{
  try
  {
    std::unique_lock<std::mutex> lock(_latest._mtx);
    if(_latest._buffer.empty())
    {
      lock.unlock();
      return false;
    }
    _current = _latest._buffer.front();
    _latest._buffer.pop();
    lock.unlock();

    cv::cvtColor(_current._mat,_current._mat,CV_BGR2HSV);
    cv::Mat out_temp;
    cv::inRange(_current._mat,lowthres,highthres,out_temp);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int r=0;r<out_temp.rows;r++)
    {
      uchar* M = out_temp.ptr<uchar>(r);
      for(int c=0;c<out_temp.cols;c++)
      {
        if(M[c] != 0)
        {
          temp_pcl->push_back(_current._pcl.points[r*out_temp.cols + c]);
        }
      }
    }

    pcl::toROSMsg(*temp_pcl,_points_pub);

    return true;
  }
  catch(cv::Exception &ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }
}

void hsv_node_threshold::publish()
{
  ros::Duration(1.0).sleep();
  while(ros::ok() && _pnh->ok())
  {
    if(run())
    {
      _points_pub.header.frame_id = _points_pub_frame_id.c_str();
      _kin_pub.publish(_points_pub);
    }
    _loop->sleep();
  }
  ROS_ERROR("Ending thread publish");
}
