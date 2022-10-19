#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <cmath>

#define goal_num 5
double goal_arr[goal_num][3] = {
                        /* square */
                        {1, 1, -90},
                        {10, 1, 180},
                        {10, 10, 90},
                        {1, 10, 0}
                        /* star */
                        // {5.544, 10.802, 0},
                        // {2.454, 1.291, 0},
                        // {10.544, 7.169, 0},
                        // {0.544, 7.169, 0},
                        // {8.635, 1.291, 0},
                        };
bool reached_status = true;
int num = 0;
double degree;
geometry_msgs::Pose  goal_pose;

void reached_CallBack(std_msgs::Bool msg)
{
    ROS_INFO("reached_status : %s", msg.data ? "true" : "false");
    reached_status = msg.data;
}

void call_goal_timer_Callback(const ros::TimerEvent& event, ros::Publisher goal_pub)
{
    if (reached_status){
        goal_pose.position.x = goal_arr[num][0];
        goal_pose.position.y = goal_arr[num][1];
        degree = goal_arr[num][2];
        if (degree >= 180) degree -= 360;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, degree * M_PI / 180.0);
        goal_pose.orientation = tf2::toMsg(q);
        goal_pub.publish(goal_pose);
        ROS_INFO("\nNew goal : [%f, %f, %f]", goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation));
        reached_status = false;
        num++;
        if (num >= goal_num) num = 0;
    }
    else{
        ROS_INFO("MOVING !");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_goal");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Pose>("/base_goal", 1);
    ros::Subscriber reached_sub = nh.subscribe("/reached_status", 1, reached_CallBack);
    ros::Timer call_goal_timer = nh.createTimer(ros::Duration(0.5), boost::bind(call_goal_timer_Callback, _1, goal_pub));

    ros::spin();
} 