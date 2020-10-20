#include <hsv_node_threshold/pose_extraction_pc2.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"pose_extraction_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pose_extraction_pc2 test(pnh,nh);

  return 1;
}
