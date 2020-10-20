#ifndef HSV_NODE_THRESHOLD_H
#define HSV_NODE_THRESHOLD_H
#include <ros/ros.h>
#include <algorithm>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class hsv_node_threshold    //this class just filters messages so they're approximate time, and then publishes the HSV filtered pointcloud2
{
public:
  hsv_node_threshold(ros::NodeHandle pnh,ros::NodeHandle nh);

private:
  struct mat_pcl_container
  {
    cv::Mat _mat;
    pcl::PointCloud<pcl::PointXYZRGB> _pcl;
  };

  struct container_buffer
  {
    std::queue<mat_pcl_container> _buffer;
    std::mutex _mtx;
  };

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image>  syncpol;

  ros::NodeHandle* _pnh;
  ros::Rate* _loop;
  ros::Rate* _filter_loop;
  message_filters::Synchronizer<syncpol>* _sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* _kin_sub;
  message_filters::Subscriber<sensor_msgs::Image>* _rgb_sub;

  ros::Publisher _kin_pub;   //publishes the thresholded pointcloud2 so that another node subscribes to it for euclidean/RANSAC

  cv_bridge::CvImageConstPtr _img;

  container_buffer _latest;
  mat_pcl_container _current;
  sensor_msgs::PointCloud2 _points_pub;
  std::thread* pub_thread;

  std::string _rgb_sub_name,_kin_sub_name,_points_pub_name,_points_pub_frame_id;
  double _pub_rate;
  cv::Scalar lowthres,highthres;

  void publish();
  bool run();
  bool setup(ros::NodeHandle nh);
  void filteredCB(const sensor_msgs::PointCloud2ConstPtr &kin, const sensor_msgs::ImageConstPtr &rgb);
};

#endif // HSV_NODE_THRESHOLD_H
