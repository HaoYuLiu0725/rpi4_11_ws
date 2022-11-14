#include <arm_move/arm_move.h>

using namespace std;
using namespace arm_move;

ArmMove::ArmMove(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
    timer_ = nh_.createTimer(ros::Duration(1.0), &ArmMove::timerCallback, this, false, false);
    initialize();
}

bool ArmMove::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    /* get param */
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);

    get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 10);
    get_param_ok = nh_local_.param<double>("init_arm_x", p_init_arm_x, 0.0);
    get_param_ok = nh_local_.param<double>("init_arm_y", p_init_arm_y, 0.0);
    get_param_ok = nh_local_.param<double>("init_arm_z", p_init_arm_z, 0.0);
    get_param_ok = nh_local_.param<double>("storage1_x", p_storage1_x, 0.0);
    get_param_ok = nh_local_.param<double>("storage1_y", p_storage1_y, 0.0);
    get_param_ok = nh_local_.param<double>("storage1_z", p_storage1_z, 0.0);
    get_param_ok = nh_local_.param<double>("storage2_x", p_storage2_x, 0.0);
    get_param_ok = nh_local_.param<double>("storage2_y", p_storage2_y, 0.0);
    get_param_ok = nh_local_.param<double>("storage2_z", p_storage2_z, 0.0);

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("mission_goal_topic", p_mission_goal_topic_, "/mission_goal");
    get_param_ok = nh_local_.param<string>("mission_topic", p_mission_topic_, "/mission");
    get_param_ok = nh_local_.param<string>("mission_status_topic", p_mission_status_topic_, "/mission_status");
    get_param_ok = nh_local_.param<string>("arm_goal_topic", p_arm_goal_topic_, "/arm_goal");
    get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");
    get_param_ok = nh_local_.param<string>("arm_status_topic", p_arm_status_topic_, "/arm_status");

    /* check param */
    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Arm Move]: " << "param set ok");
    }
    else
    {
        ROS_WARN_STREAM("[Arm Move]: " << "param set fail");
    }

    /* ros node param */
    timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            mission_goal_sub_ = nh_.subscribe(p_mission_goal_topic_, 10, &ArmMove::missionGoalCallback, this);
            mission_sub_ = nh_.subscribe(p_mission_topic_, 10, &ArmMove::missionCallback, this);
            mission_status_pub_ = nh_.advertise<std_msgs::Bool>(p_mission_status_topic_, 10);
            arm_goal_pub_ = nh_.advertise<geometry_msgs::Point>(p_arm_goal_topic_, 10);
            suck_pub_ = nh_.advertise<std_msgs::Bool>(p_suck_topic_, 10);
            arm_status_sub_ = nh_.subscribe(p_arm_status_topic_, 10, &ArmMove::armStatusCallback, this);
            timer_.start();
        }
        else
        {
            mission_goal_sub_.shutdown();
            mission_sub_.shutdown();
            mission_status_pub_.shutdown();
            arm_goal_pub_.shutdown();
            suck_pub_.shutdown();
            arm_status_sub_.shutdown();
            timer_.stop();
        }
    }

    /* init state param */
    storage_1.x = p_storage1_x;
    storage_1.y = p_storage1_y;
    storage_1.z = p_storage1_z;
    storage_2.x = p_storage2_x;
    storage_2.y = p_storage2_y;
    storage_2.z = p_storage2_z;

    // output_twist_.linear.x = 0;
    // output_twist_.linear.y = 0;
    // output_twist_.angular.z = 0;
    // reached_status_.data = true;

    publish();

    return true;
}