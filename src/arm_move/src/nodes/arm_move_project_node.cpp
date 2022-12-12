#include "arm_move/arm_move_project.h"

using namespace arm_move_project;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_move_project");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Arm Move Project]: Initializing node");
    ArmMoveProject arm_move_project(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Arm Move Project]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Arm Move Project]: Unexpected error");
  }

  return 0;
}
