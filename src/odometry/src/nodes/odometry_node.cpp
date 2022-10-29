#include "odometry/odometry.h"

using namespace odometry;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Odometry]: Initializing node");
    Odometry odometry(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Odometry]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Odometry]: Unexpected error");
  }

  return 0;
}
