#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <cmath>

double now_x, now_y, now_theta;
double goal_x, goal_y, goal_theta;
double linear_velocity, angular_velocity;

double MAX_linear_speed  = 1.6; // m/s
double MAX_angular_speed = 1.2; // rad/s
double linear_acceleration  = 0.4; // m/s^2
double linear_deceleration  = 0.4; // m/s^2
double angular_acceleration = 0.3; // rad/s^2
double angular_deceleration = 0.3; // rad/s^2

double t_linear_speed = 0; // m/s, target linear speed
double t_angular_speed = 0; // rad/s, target angular speed
bool have_new_goal = false;
std_msgs::Bool reached_status;

enum Move_State
{
  LINEAR,
  STOP_LINEAR,
  TURN,
  STOP_TURN,
};
Move_State move_state = LINEAR;

enum Speed_State
{
  ACCELERATE,
  MAX_SPEED,
  DECELERATE,
  STOP,
};
Speed_State speed_state = ACCELERATE;

void pose_CallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
    now_x = pose->pose.pose.position.x;
    now_y = pose->pose.pose.position.y;
    now_theta = tf2::getYaw(pose->pose.pose.orientation);
    // ROS_INFO("\nnow pose: %f, %f, %f", now_x, now_y, now_theta);
}

// void pose_CallBack(const geometry_msgs::PoseConstPtr& pose)
// {
//     now_x = pose->position.x;
//     now_y = pose->position.y;
//     now_theta = tf2::getYaw(pose->orientation);
//     // ROS_INFO("\nnow pose: %f, %f, %f", now_x, now_y, now_theta);
// }

void speed_CallBack(const geometry_msgs::TwistConstPtr& speed)
{
    linear_velocity = sqrt(pow(speed->linear.x, 2) + pow(speed->linear.y, 2));
    angular_velocity = speed->angular.z;
    // ROS_INFO("\nnow speed: %f, %f", linear_velocity, angular_velocity);
}

void goal_CallBack(const geometry_msgs::PoseConstPtr& pose)
{
    goal_x = pose->position.x;
    goal_y = pose->position.x;
    goal_theta = tf2::getYaw(pose->orientation);
    ROS_INFO("\nNew goal : [%f, %f, %f]", goal_x, goal_y, goal_theta);
    have_new_goal = true;
    move_state = LINEAR;
    speed_state = ACCELERATE;
}

void twist_publish(ros::Publisher vel_pub, double Vx, double Vy, double W)
{
    geometry_msgs::Twist twist;
    twist.linear.x = Vx;
    twist.linear.y = Vy;
    twist.angular.z = W;
    vel_pub.publish(twist);
}

bool hasReachedGoal_XY()
{
  return fabsf(now_x - goal_x) < 0.1 && fabsf(now_y - goal_y) < 0.1;
}

bool hasReachedGoal_Theta()
{
  return fabsf(now_theta - goal_theta) < 0.01;
}

bool hasStopped()
{
  return fabsf(angular_velocity) < 0.0001 && fabsf(linear_velocity) < 0.0001;
}

/*-------speed_timer_Callback------------------------------------------------------------*/
bool time_to_decelerate(double &speed, double deceleration)
{
    double decelerate_distance = pow(speed, 2) / (2 * deceleration);
    ROS_INFO("decelerate_distance: %f", decelerate_distance);
    double remain_distance;
    if (move_state == LINEAR){
        remain_distance = sqrt(pow(goal_x - now_x, 2) + pow(goal_y - now_y, 2));
    }
    else if (move_state == TURN){
        remain_distance = fabsf(goal_theta - now_theta);
    }
    ROS_INFO("remain_distance: %f", remain_distance);
    return (remain_distance <= decelerate_distance);
}

void accelerate(double &speed, double MAX_speed, double acceleration, double deceleration)
{
    if (time_to_decelerate(speed, deceleration)){
        speed_state = DECELERATE;
    }
    else if (fabsf(speed - MAX_speed) < 0.01){
        speed_state = MAX_SPEED;
    }
    else{
        ROS_INFO("accelerate");
        speed += acceleration * 0.05; //speed_timer period = 0.05 seconds
    }
}

void max_speed(double &speed, double MAX_speed, double acceleration, double deceleration)
{
    if (time_to_decelerate(speed, deceleration)){
        speed_state = DECELERATE;
    }
    else{
        ROS_INFO("max_speed");
        speed = MAX_speed;
    }
}

void decelerate(double &speed, double MAX_speed, double acceleration, double deceleration)
{
    if (hasStopped()){
        speed_state = STOP;
    }
    else{
        ROS_INFO("decelerate");
        speed -= deceleration * 0.05; //speed_timer period = 0.05 seconds
    }
}
void stop(double &speed)
{
    if (hasStopped()){
        ROS_INFO("stop !");
        speed_state = ACCELERATE;
    }
    else{
        speed = 0.0;
    }
}

void speed_timer_Callback(const ros::TimerEvent& event, ros::Publisher reached_pub)
{
    if(have_new_goal){
        if (hasReachedGoal_XY() && hasReachedGoal_Theta()){
            reached_status.data = true;
            reached_pub.publish(reached_status);
            have_new_goal = false;
            ROS_INFO("\nspeed reached_status : %s", reached_status.data ? "true" : "false");
            t_linear_speed = 0.0;
            t_angular_speed = 0.0;
            speed_state = ACCELERATE;
        }

        if (move_state == LINEAR){
            if (speed_state == ACCELERATE){
                accelerate(t_linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(t_linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == DECELERATE){
                decelerate(t_linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == STOP){
                stop(t_linear_speed);
            }
            ROS_INFO("t_linear_speed: %f", t_linear_speed);
        }
        else if (move_state == TURN){
            if (speed_state == ACCELERATE){
                accelerate(t_angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(t_angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == DECELERATE){
                decelerate(t_angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == STOP){
                stop(t_angular_speed);
            }
            ROS_INFO("t_angular_speed: %f", t_angular_speed);
        }
    }
}

/*-------move_timer_Callback------------------------------------------------------------*/
void linear(ros::Publisher vel_pub)
{
    if (hasReachedGoal_XY()){
        move_state = STOP_LINEAR;
        twist_publish(vel_pub, 0, 0, 0);
    }
    else{
        double angle = atan2( goal_y - now_y , goal_x - now_x ) - now_theta;
        double Vx = t_linear_speed * cos(angle);
        double Vy = t_linear_speed * sin(angle);
        twist_publish(vel_pub, Vx, Vy, 0);
    }
}

void stopLinear(ros::Publisher vel_pub)
{
    if (hasStopped()){
        ROS_INFO("Reached goal_XY !");
        move_state = TURN;
    }
    else{
        twist_publish(vel_pub, 0, 0, 0);
    }
}

void turn(ros::Publisher vel_pub)
{
    if (hasReachedGoal_Theta()){
        move_state = STOP_TURN;
        twist_publish(vel_pub, 0, 0, 0);
    }
    else{
        double W = t_angular_speed;
        double error = goal_theta - now_theta;
        if (error < 0) W = -t_angular_speed;
        if (abs(error) > M_PI) W = -W;
        twist_publish(vel_pub, 0, 0, W);
    }
}

void stopTurn(ros::Publisher vel_pub)
{
    if (hasStopped()){
        ROS_INFO("Reached goal_Theta !");
        move_state = LINEAR;
    }
    else{
        twist_publish(vel_pub, 0, 0, 0);
    }
}

void move_timer_Callback(const ros::TimerEvent& event, ros::Publisher vel_pub, ros::Publisher reached_pub)
{
    if(have_new_goal){
        if (hasReachedGoal_XY() && hasReachedGoal_Theta()){
            reached_status.data = true;
            reached_pub.publish(reached_status);
            have_new_goal = false;
            ROS_INFO("\nmove reached_status : %s", reached_status.data ? "true" : "false");
        }

        if (move_state == LINEAR){
            linear(vel_pub);
        }
        else if (move_state == STOP_LINEAR){
            stopLinear(vel_pub);
        }
        else if (move_state == TURN){
            turn(vel_pub);
        }
        else if (move_state == STOP_TURN){
            stopTurn(vel_pub);
        }
    }
    else{
        twist_publish(vel_pub, 0, 0, 0);
    }
}

int main(int argc, char **argv)
{
    //node initialization
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber pose_sub = nh.subscribe("/odom_pose", 1, &pose_CallBack);
    ros::Subscriber speed_sub = nh.subscribe("/base_speed", 1, &speed_CallBack);
    ros::Publisher reached_pub = nh.advertise<std_msgs::Bool>("/reached_status", 1);
    ros::Subscriber goal_sub = nh.subscribe("/base_goal", 1, &goal_CallBack);

    // Timer
    ros::Timer move_timer = nh.createTimer(ros::Duration(0.016), boost::bind(move_timer_Callback, _1, vel_pub, reached_pub));
    ros::Timer speed_timer = nh.createTimer(ros::Duration(0.05), boost::bind(speed_timer_Callback, _1, reached_pub));

    ros::spin();
}