#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <cmath>

#define goal_num 7
double goal_arr[goal_num][3] = {
                        /* LEVEL 2*/
                        {0.5, -0.15, 90},
                        {0.65, -0.15, 90},
                        {0.65, 1.25, 90},
                        {0.25, 1.25, 90},
                        {0.25, 2.35, 90},
                        {0.50, 2.35, 90},
                        {0.50, 3.275, 90},
                        };
bool reached_status = true;
int num = 0;
double degree;
geometry_msgs::PoseStamped  goal_pose;

void reached_CallBack(std_msgs::Bool msg)
{
    ROS_INFO("reached_status : %s", msg.data ? "true" : "false");
    reached_status = msg.data;
}

void call_goal_timer_Callback(const ros::TimerEvent& event, ros::Publisher goal_pub)
{
    if (reached_status){
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = goal_arr[num][0];
        goal_pose.pose.position.y = goal_arr[num][1];
        degree = goal_arr[num][2];
        if (degree >= 180) degree -= 360;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, degree * M_PI / 180.0);
        goal_pose.pose.orientation = tf2::toMsg(q);
        goal_pub.publish(goal_pose);
        ROS_INFO("\nNew goal : [%f, %f, %f]", goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
        reached_status = false;
        num++;
        if (num >= goal_num) num = goal_num-1;
    }
    else{
        ROS_INFO("MOVING !");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_goal");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/base_goal", 10);
    ros::Subscriber reached_sub = nh.subscribe("/reached_status", 10, reached_CallBack);
    ros::Timer call_goal_timer = nh.createTimer(ros::Duration(0.5), boost::bind(call_goal_timer_Callback, _1, goal_pub));

    ros::spin();
} 