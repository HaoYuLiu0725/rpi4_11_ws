#include <base_move/my_navigation.h>

using namespace my_navigation;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_navigation");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[My_navigation]: Initializing node");
    My_navigation my_navigation(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[My_navigation]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[My_navigation]: Unexpected error");
  }

  return 0;
}
