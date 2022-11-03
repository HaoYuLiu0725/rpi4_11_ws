#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
namespace my_navigation
{
class My_navigation
{
public:
    My_navigation(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void mapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr);
    void twistPublish(double Vx, double Vy, double W);
    void transformCallback(const ros::TimerEvent& e);
    void moveTimerCallback(const ros::TimerEvent& e);
    void speedTimerCallback(const ros::TimerEvent& e);
    /* check status*/
    bool hasReachedGoal_XY();
    bool hasReachedGoal_Theta();
    bool hasStopped();
    bool timeToDecelerate(double &speed, double deceleration);
    /* move_timer */
    void linear();
    void stopLinear();
    void turn();
    void stopTurn();
    /* speed_timer */
    void accelerate(double &speed, double deceleration, double MAX_speed, double acceleration);
    void max_speed(double &speed, double deceleration, double MAX_speed);
    void decelerate(double &speed, double deceleration);
    void stop(double &speed);

    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer move_timer_;
    ros::Timer speed_timer_;

    /* ros inter-node */
    ros::Publisher twist_pub_;
    ros::Publisher reached_pub_;
    ros::Subscriber mapPose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;

    std_msgs::Bool reached_status_;
    geometry_msgs::Twist output_twist_;

    ros::Time last_time_;
    ros::Duration timeout_;

    /* ros param */
    bool p_active_;

    double p_move_frequency_;
    double p_speed_frequency_;
    double p_transform_frequency_;
    double p_init_pose_x;
    double p_init_pose_y;
    double p_init_pose_yaw;
    double p_MAX_linear_speed_;     // m/s
    double p_MAX_angular_speed_;    // rad/s
    double p_linear_acceleration_;  // m/s^2
    double p_linear_deceleration_;  // m/s^2
    double p_angular_acceleration_; // rad/s^2
    double p_angular_deceleration_; // rad/s^2
    double p_linear_margin_;    // m
    double p_angular_margin_;   // rad
    double p_stop_margin_;      // m/s & rad/s

    std::string p_twist_topic_;
    std::string p_reached_topic_;
    std::string p_mapPose_topic_;
    std::string p_odom_topic_;
    std::string p_goal_topic_;

    /* param */
    bool have_new_goal;
    double now_x, now_y, now_theta;
    double goal_x, goal_y, goal_theta;
    double linear_velocity, angular_velocity;
    double t_linear_speed; // m/s, target linear speed
    double t_angular_speed; // rad/s, target angular speed

    /* state */
    enum Move_State
    {
        LINEAR,
        STOP_LINEAR,
        TURN,
        STOP_TURN,
    }move_state;
    enum Speed_State
    {
        ACCELERATE,
        MAX_SPEED,
        DECELERATE,
        STOP,
    }speed_state;
};
}  // namespace odometry
