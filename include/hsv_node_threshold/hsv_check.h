#ifndef HSV_CHECK_H
#define HSV_CHECK_H
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class hsv_check
{
public:
  hsv_check(ros::NodeHandle pnh);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> _pol;
  ros::NodeHandle* _pnh;
  ros::Publisher _kin_pub;

  struct _buffer
  {
    cv::Mat _mat;
    pcl::PointCloud<pcl::PointXYZRGB> _pcl;
  };

  message_filters::Subscriber<sensor_msgs::PointCloud2>* _kin_sub;
  message_filters::Subscriber<sensor_msgs::Image>* _rgb_sub;
  message_filters::Synchronizer<_pol>* _sync;

  _buffer _latest;
  sensor_msgs::PointCloud2 _publish;

  std::string _rgb_sub_topic,_kin_sub_topic,_kin_pub_topic,_kin_pub_frame;
  cv::Scalar* _hsv_lvec;
  cv::Scalar* _hsv_uvec;

  bool setup();
  void callback(const sensor_msgs::ImageConstPtr &rgb,const sensor_msgs::PointCloud2ConstPtr &kin);
};

#endif // HSV_CHECK_H
