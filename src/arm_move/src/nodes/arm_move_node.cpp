#include "arm_move/arm_move.h"

using namespace arm_move;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_move");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Arm Move]: Initializing node");
    ArmMove arm_move(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Arm Move]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Arm Move]: Unexpected error");
  }

  return 0;
}
