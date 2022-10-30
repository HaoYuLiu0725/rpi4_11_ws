#pragma once

#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

namespace path_generation
{
class Path_generation
{
public:
  Path_generation(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  void initialize()
  {
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void goalCallback(const geometry_msgs::Pose::ConstPtr& ptr);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& ptr);
  void timerCallback(const ros::TimerEvent& e);
  void updatePose();
  void generatePath();

  void publish();
  bool have_new_goal; 

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  /* ros inter-node */
  ros::Subscriber goal_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher path_pub_;

  geometry_msgs::Pose goal_pose_;
  geometry_msgs::Pose input_pose_;
  geometry_msgs::Pose now_pose_;
  nav_msgs::Path output_path_;

  ros::Time last_time_;
  ros::Duration timeout_;

  /* ros param */
  bool p_active_;

  double p_frequency_;
  double p_init_pose_x;
  double p_init_pose_y;
  double p_init_pose_yaw;
  int p_resolution_;

  std::string p_path_topic_;
  std::string p_goal_topic_;
  std::string p_pose_topic_;
  std::string p_frame_id_;
};
}  // namespace path_generation
