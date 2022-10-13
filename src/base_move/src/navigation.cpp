#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Bool.h>
#include <cmath>

turtlesim::Pose now_pose;
turtlesim::Pose goal_pose;
double MAX_linear_speed  = 1.6; // m/s
double MAX_angular_speed = 1.2; // rad/s
double linear_acceleration  = 0.4; // m/s^2
double linear_deceleration  = 0.4; // m/s^2
double angular_acceleration = 0.3; // rad/s^2
double angular_deceleration = 0.3; // rad/s^2

double linear_speed = 0; // m/s
double angular_speed = 0; // rad/s
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

void pose_CallBack(const turtlesim::PoseConstPtr& pose)
{
    now_pose.x = pose->x;
    now_pose.y = pose->y;
    now_pose.theta = pose->theta;
    now_pose.angular_velocity = pose->angular_velocity;
    now_pose.linear_velocity = pose->linear_velocity;
    // ROS_INFO("\nnow pose: %f, %f, %f", now_pose.x, now_pose.y, now_pose.theta);
}

void goal_CallBack(const turtlesim::PoseConstPtr& pose)
{
    ROS_INFO("\nNew goal : [%f, %f, %f]", pose->x, pose->y, pose->theta);
    goal_pose.x = pose->x;
    goal_pose.y = pose->y;
    goal_pose.theta = pose->theta;
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
  return fabsf(now_pose.x - goal_pose.x) < 0.1 && fabsf(now_pose.y - goal_pose.y) < 0.1;
}

bool hasReachedGoal_Theta()
{
  return fabsf(now_pose.theta - goal_pose.theta) < 0.01;
}

bool hasStopped()
{
  return fabsf(now_pose.angular_velocity) < 0.0001 && fabsf(now_pose.linear_velocity) < 0.0001;
}

/*-------speed_timer_Callback------------------------------------------------------------*/
bool time_to_decelerate(double &speed, double deceleration)
{
    double decelerate_distance = pow(speed, 2) / (2 * deceleration);
    ROS_INFO("decelerate_distance: %f", decelerate_distance);
    double remain_distance;
    if (move_state == LINEAR){
        remain_distance = sqrt(pow(goal_pose.x - now_pose.x, 2) + pow(goal_pose.y - now_pose.y, 2));
    }
    else if (move_state == TURN){
        remain_distance = fabsf(goal_pose.theta - now_pose.theta);
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
            linear_speed = 0.0;
            angular_speed = 0.0;
            speed_state = ACCELERATE;
        }

        if (move_state == LINEAR){
            if (speed_state == ACCELERATE){
                accelerate(linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == DECELERATE){
                decelerate(linear_speed, MAX_linear_speed, linear_acceleration, linear_deceleration);
            }
            else if (speed_state == STOP){
                stop(linear_speed);
            }
            ROS_INFO("linear_speed: %f", linear_speed);
        }
        else if (move_state == TURN){
            if (speed_state == ACCELERATE){
                accelerate(angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == MAX_SPEED){
                max_speed(angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == DECELERATE){
                decelerate(angular_speed, MAX_angular_speed, angular_acceleration, angular_deceleration);
            }
            else if (speed_state == STOP){
                stop(angular_speed);
            }
            ROS_INFO("angular_speed: %f", angular_speed);
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
        double angle = atan2( goal_pose.y - now_pose.y , goal_pose.x - now_pose.x ) - now_pose.theta;
        double Vx = linear_speed * cos(angle);
        double Vy = linear_speed * sin(angle);
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
        double W = angular_speed;
        double error = goal_pose.theta - now_pose.theta;
        if (error < 0) W = -angular_speed;
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
    ros::Subscriber pose_sub = nh.subscribe("/odom_pose", 1, pose_CallBack);
    ros::Publisher reached_pub = nh.advertise<std_msgs::Bool>("/reached_status", 1);
    ros::Subscriber goal_sub = nh.subscribe("/base_goal", 1, goal_CallBack);

    // Timer
    ros::Timer move_timer = nh.createTimer(ros::Duration(0.016), boost::bind(move_timer_Callback, _1, vel_pub, reached_pub));
    ros::Timer speed_timer = nh.createTimer(ros::Duration(0.05), boost::bind(speed_timer_Callback, _1, reached_pub));

    ros::spin();
}