#include <hsv_node_threshold/hsv_node_threshold.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"hsv_threshold_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  hsv_node_threshold test(pnh,nh);

  return -1;
}
