#include <hsv_node_threshold/hsv_check.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"hsv_value_check");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  hsv_check test(pnh);

  return 1;
}
