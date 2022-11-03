#include <base_move/my_navigation.h>

using namespace std;
using namespace my_navigation;

My_navigation::My_navigation(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
    move_timer_ = nh_.createTimer(ros::Duration(1.0), &My_navigation::moveTimerCallback, this, false, false);
    speed_timer_ = nh_.createTimer(ros::Duration(1.0), &My_navigation::speedTimerCallback, this, false, false);
    initialize();
}

bool My_navigation::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    /* get param */
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);

    get_param_ok = nh_local_.param<double>("move_frequency", p_move_frequency_, 60);
    get_param_ok = nh_local_.param<double>("speed_frequency", p_speed_frequency_, 100);
    get_param_ok = nh_local_.param<double>("init_pose_x", p_init_pose_x, 0.0);
    get_param_ok = nh_local_.param<double>("init_pose_y", p_init_pose_y, 0.0);
    get_param_ok = nh_local_.param<double>("init_pose_yaw", p_init_pose_yaw, 0.0);
    get_param_ok = nh_local_.param<double>("MAX_linear_speed", p_MAX_linear_speed_, 0.5);           // m/s
    get_param_ok = nh_local_.param<double>("MAX_angular_speed", p_MAX_angular_speed_, 0.8);         // rad/s
    get_param_ok = nh_local_.param<double>("linear_acceleration", p_linear_acceleration_, 0.05);    // m/s^2
    get_param_ok = nh_local_.param<double>("linear_deceleration", p_linear_deceleration_, 0.05);    // m/s^2
    get_param_ok = nh_local_.param<double>("angular_acceleration", p_angular_acceleration_, 0.4);   // rad/s^2
    get_param_ok = nh_local_.param<double>("angular_deceleration", p_angular_deceleration_, 0.4);   // rad/s^2
    get_param_ok = nh_local_.param<double>("linear_margin", p_linear_margin_, 0.001);   // m
    get_param_ok = nh_local_.param<double>("angular_margin", p_angular_margin_, 0.001); // rad
    get_param_ok = nh_local_.param<double>("stop_margin", p_stop_margin_, 0.0001);      // m/s & rad/s

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("twist_topic", p_twist_topic_, "/cmd_vel");
    get_param_ok = nh_local_.param<string>("reached_topic", p_reached_topic_, "/reached_status");
    get_param_ok = nh_local_.param<string>("mapPose_topic", p_mapPose_topic_, "/map_pose");
    get_param_ok = nh_local_.param<string>("odom_topic", p_odom_topic_, "/odom");
    get_param_ok = nh_local_.param<string>("goal_topic", p_goal_topic_, "/base_goal");

    /* check param */
    if (get_param_ok)
    {
        ROS_INFO_STREAM("[My_navigation]: "
                        << "param set ok");
    }
    else
    {
        ROS_WARN_STREAM("[My_navigation]: "
                        << "param set fail");
    }

    /* ros node param */
    move_timer_.setPeriod(ros::Duration(1 / p_move_frequency_), false);
    speed_timer_.setPeriod(ros::Duration(1 / p_speed_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>(p_twist_topic_, 10);
            reached_pub_ = nh_.advertise<std_msgs::Bool>(p_reached_topic_, 10);
            mapPose_sub_ = nh_.subscribe(p_mapPose_topic_, 10, &My_navigation::mapPoseCallback, this);
            odom_sub_ = nh_.subscribe(p_odom_topic_, 10, &My_navigation::odomCallback, this);
            goal_sub_ = nh_.subscribe(p_goal_topic_, 10, &My_navigation::goalCallback, this);
            move_timer_.start();
            speed_timer_.start();
        }
        else
        {
            twist_pub_.shutdown();
            reached_pub_.shutdown();
            odom_sub_.shutdown();
            goal_sub_.shutdown();
            move_timer_.stop();
            speed_timer_.stop();
        }
    }

    /* init state param */
    now_x = p_init_pose_x;
    now_y = p_init_pose_y;
    now_theta = p_init_pose_yaw;
    bool have_new_goal = false;
    t_linear_speed = 0;
    t_angular_speed = 0;
    move_state = LINEAR;
    speed_state = ACCELERATE;

    output_twist_.linear.x = 0;
    output_twist_.linear.y = 0;
    output_twist_.angular.z = 0;
    reached_status_.data = true;

    twistPublish(0, 0, 0);

    return true;
}
void My_navigation::mapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr)
{
    /* get pose from transform : map to baselink*/
    now_x = pose_ptr->pose.position.x;
    now_y = pose_ptr->pose.position.y;
    now_theta = tf2::getYaw(pose_ptr->pose.orientation);
    // ROS_INFO_STREAM("[Now pose]:" << now_x << "," << now_y << "," << now_theta);

}
void My_navigation::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr)
{    
    /* get pose from odom*/
    // now_x = odom_ptr->pose.pose.position.x;
    // now_y = odom_ptr->pose.pose.position.y;
    // now_theta = tf2::getYaw(odom_ptr->pose.pose.orientation);
    // ROS_INFO_STREAM("[Now pose]:" << now_x << "," << now_y << "," << now_theta);

    /* get speed from odom*/
    linear_velocity = sqrt(pow(odom_ptr->twist.twist.linear.x, 2) + pow(odom_ptr->twist.twist.linear.y, 2));
    angular_velocity = odom_ptr->twist.twist.angular.z;
    // ROS_INFO_STREAM("[Now speed]:" << linear_velocity << "," << angular_velocity);
}

void My_navigation::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr)
{
    goal_x = pose_ptr->pose.position.x;
    goal_y = pose_ptr->pose.position.y;
    goal_theta = tf2::getYaw(pose_ptr->pose.orientation);
    ROS_INFO_STREAM("[New goal!]:" << goal_x << "," << goal_y << "," << goal_theta);
    have_new_goal = true;
    move_state = LINEAR;
    speed_state = ACCELERATE;
}

void My_navigation::twistPublish(double Vx, double Vy, double W)
{
    output_twist_.linear.x = Vx;
    output_twist_.linear.y = Vy;
    output_twist_.angular.z = W;
    twist_pub_.publish(output_twist_);
}
/*--------------------check status--------------------*/
bool My_navigation::hasReachedGoal_XY()
{
  return fabsf(now_x - goal_x) < p_linear_margin_ && fabsf(now_y - goal_y) < p_linear_margin_;
}

bool My_navigation::hasReachedGoal_Theta()
{
  return fabsf(now_theta - goal_theta) < p_angular_margin_;
}

bool My_navigation::hasStopped()
{
  return fabsf(angular_velocity) < p_stop_margin_ && fabsf(linear_velocity) < p_stop_margin_;
}

bool My_navigation::timeToDecelerate(double &speed, double deceleration)
{
    double decelerate_distance = pow(speed, 2) / (2 * deceleration);
    // ROS_INFO("decelerate_distance: %f", decelerate_distance);
    double remain_distance;
    if (move_state == LINEAR){
        remain_distance = sqrt(pow(goal_x - now_x, 2) + pow(goal_y - now_y, 2));
    }
    else if (move_state == TURN){
        remain_distance = fabsf(goal_theta - now_theta);
    }
    // ROS_INFO("remain_distance: %f", remain_distance);
    return (remain_distance <= decelerate_distance);
}

/*--------------------move_timer--------------------*/
void My_navigation::linear()
{
    if (hasReachedGoal_XY()){
        move_state = STOP_LINEAR;
        twistPublish(0, 0, 0);
    }
    else{
        double angle = atan2( goal_y - now_y , goal_x - now_x ) - now_theta;
        double Vx = t_linear_speed * cos(angle);
        double Vy = t_linear_speed * sin(angle);
        twistPublish(Vx, Vy, 0);
    }
}

void My_navigation::stopLinear()
{
    if (hasStopped()){
        ROS_INFO_STREAM("[Reached goal_XY !]");
        move_state = TURN;
    }
    else{
        twistPublish(0, 0, 0);
    }
}

void My_navigation::turn()
{
    if (hasReachedGoal_Theta()){
        move_state = STOP_TURN;
        twistPublish(0, 0, 0);
    }
    else{
        double W = t_angular_speed;
        double error = goal_theta - now_theta;
        if (error < 0) W = -t_angular_speed;
        if (abs(error) > M_PI) W = -W;
        twistPublish(0, 0, W);
    }
}

void My_navigation::stopTurn()
{
    if (hasStopped()){
        ROS_INFO_STREAM("[Reached goal_Theta !]");
        move_state = LINEAR;
    }
    else{
        twistPublish(0, 0, 0);
    }
}

void My_navigation::moveTimerCallback(const ros::TimerEvent& e)
{
    if(have_new_goal){
        if (hasReachedGoal_XY() && hasReachedGoal_Theta()){
            reached_status_.data = true;
            reached_pub_.publish(reached_status_);
            have_new_goal = false;
            ROS_INFO_STREAM("[move reached_status:]" << (reached_status_.data ? "true" : "false"));
        }

        if (move_state == LINEAR){
            linear();
        }
        else if (move_state == STOP_LINEAR){
            stopLinear();
        }
        else if (move_state == TURN){
            turn();
        }
        else if (move_state == STOP_TURN){
            stopTurn();
        }
    }
    else{
        twistPublish(0, 0, 0);
    }
}

/*--------------------speed_timer---------------------------*/
void My_navigation::accelerate(double &speed, double deceleration, double MAX_speed, double acceleration)
{
    if (timeToDecelerate(speed, deceleration)){
        speed_state = DECELERATE;
    }
    else if (fabsf(speed - MAX_speed) < 0.01){
        speed_state = MAX_SPEED;
    }
    else{
        // ROS_INFO("accelerate");
        speed += acceleration / p_speed_frequency_;
    }
}

void My_navigation::max_speed(double &speed, double deceleration, double MAX_speed)
{
    if (timeToDecelerate(speed, deceleration)){
        speed_state = DECELERATE;
    }
    else{
        // ROS_INFO("max_speed");
        speed = MAX_speed;
    }
}

void My_navigation::decelerate(double &speed, double deceleration)
{
    if (hasStopped()){
        speed_state = STOP;
    }
    else{
        // ROS_INFO("decelerate");
        speed -= deceleration / p_speed_frequency_;
    }
}
void My_navigation::stop(double &speed)
{
    if (hasStopped()){
        // ROS_INFO("stop !");
        speed_state = ACCELERATE;
    }
    else{
        speed = 0.0;
    }
}

void My_navigation::speedTimerCallback(const ros::TimerEvent& e)
{
    if(have_new_goal){
        if (hasReachedGoal_XY() && hasReachedGoal_Theta()){
            reached_status_.data = true;
            reached_pub_.publish(reached_status_);
            have_new_goal = false;
            ROS_INFO_STREAM("[move reached_status:]" << (reached_status_.data ? "true" : "false"));
            t_linear_speed = 0.0;
            t_angular_speed = 0.0;
            speed_state = ACCELERATE;
        }

        if (move_state == LINEAR){
            if (speed_state == ACCELERATE){
                accelerate(t_linear_speed, p_linear_deceleration_, p_MAX_linear_speed_, p_linear_acceleration_);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(t_linear_speed, p_linear_deceleration_, p_MAX_linear_speed_);
            }
            else if (speed_state == DECELERATE){
                decelerate(t_linear_speed, p_linear_deceleration_);
            }
            else if (speed_state == STOP){
                stop(t_linear_speed);
            }
            // ROS_INFO("t_linear_speed: %f", t_linear_speed);
        }
        else if (move_state == TURN){
            if (speed_state == ACCELERATE){
                accelerate(t_angular_speed, p_angular_deceleration_, p_MAX_angular_speed_, p_angular_acceleration_);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(t_angular_speed, p_angular_deceleration_, p_MAX_angular_speed_);
            }
            else if (speed_state == DECELERATE){
                decelerate(t_angular_speed, p_angular_deceleration_);
            }
            else if (speed_state == STOP){
                stop(t_angular_speed);
            }
            // ROS_INFO("t_angular_speed: %f", t_angular_speed);
        }
    }
}