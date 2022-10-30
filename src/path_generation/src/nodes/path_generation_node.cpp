#include "path_generation/path_generation.h"

using namespace path_generation;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_generation");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Path_generation]: Initializing node");
    Path_generation path_generation(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Path_generation]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Path_generation]: Unexpected error");
  }

  return 0;
}
