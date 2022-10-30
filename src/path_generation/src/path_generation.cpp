#include "path_generation/path_generation.h"

using namespace std;
using namespace path_generation;

Path_generation::Path_generation(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
  timer_ = nh_.createTimer(ros::Duration(1.0), &Path_generation::timerCallback, this, false, false);
  initialize();
}

bool Path_generation::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  /* get param */
  get_param_ok = nh_local_.param<bool>("active", p_active_, true);

  get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 30);
  get_param_ok = nh_local_.param<double>("init_pose_x", p_init_pose_x, 0.0);
  get_param_ok = nh_local_.param<double>("init_pose_y", p_init_pose_y, 0.0);
  get_param_ok = nh_local_.param<double>("init_pose_yaw", p_init_pose_yaw, 0.0);
  get_param_ok = nh_local_.param<int>("resolution", p_resolution_, 100);

  double timeout;
  get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
  timeout_.fromSec(timeout);

  get_param_ok = nh_local_.param<string>("path_topic", p_path_topic_, "nav_path");
  get_param_ok = nh_local_.param<string>("goal_topic", p_goal_topic_, "nav_goal");
  get_param_ok = nh_local_.param<string>("pose_topic", p_pose_topic_, "ekf_pose");
  get_param_ok = nh_local_.param<string>("frame_id", p_frame_id_, "map");

  /* check param */
  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Path_generation]: "
                    << "param set ok");
  }
  else
  {
    ROS_WARN_STREAM("[Path_generation]: "
                    << "param set fail");
  }

  /* ros node param */
  timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      goal_sub_ = nh_.subscribe(p_goal_topic_, 10, &Path_generation::goalCallback, this);
      pose_sub_ = nh_.subscribe(p_pose_topic_, 10, &Path_generation::poseCallback, this);
      path_pub_ = nh_.advertise<nav_msgs::Path>(p_path_topic_, 10);
      timer_.start();
    }
    else
    {
      goal_sub_.shutdown();
      pose_sub_.shutdown();
      path_pub_.shutdown();
      timer_.stop();
    }
  }

  /* init state param */
  now_pose_.position.x = p_init_pose_x;
  now_pose_.position.y = p_init_pose_y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p_init_pose_yaw);
  now_pose_.orientation = tf2::toMsg(q);

  return true;
}

void Path_generation::goalCallback(const geometry_msgs::Pose::ConstPtr& ptr)
{
  goal_pose_ = *ptr;
  ROS_INFO_STREAM("[Path_generation]: "<< "New goal received!");
  have_new_goal = true;
}

void Path_generation::poseCallback(const nav_msgs::Odometry::ConstPtr& ptr)
{
  input_pose_ = ptr->pose.pose;
  last_time_ = ros::Time::now();
}

void Path_generation::timerCallback(const ros::TimerEvent& e)
{
  if(ros::Time::now().toSec() - last_time_.toSec() > timeout_.toSec()){
    return;
  }
  if(have_new_goal){
    updatePose();
    generatePath();
    publish();
  }
}

void Path_generation::updatePose()
{
  now_pose_ = input_pose_;
}

void Path_generation::generatePath()
{
  geometry_msgs::PoseStamped new_pose;
  double now_yaw, goal_yaw, new_yaw;
  for(int i = 0; i < p_resolution_; i++){
    new_pose.pose.position.x = now_pose_.position.x + (i / p_resolution_) * (goal_pose_.position.x - now_pose_.position.x);
    new_pose.pose.position.y = now_pose_.position.y + (i / p_resolution_) * (goal_pose_.position.y - now_pose_.position.y);
    now_yaw = tf2::getYaw(now_pose_.orientation);
    goal_yaw = tf2::getYaw(goal_pose_.orientation);
    new_yaw = now_yaw + (i / p_resolution_) * (goal_yaw - now_yaw);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, new_yaw);
    new_pose.pose.orientation = tf2::toMsg(q);
    output_path_.poses.push_back(new_pose);
  }
}

void Path_generation::publish()
{
  ros::Time now = ros::Time::now();
  output_path_.header.stamp = now;
  output_path_.header.frame_id = p_frame_id_;
  
  path_pub_.publish(output_path_);
  have_new_goal = false;
}
