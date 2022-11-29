#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <iostream>

double now_x, now_y, now_theta;

void mapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr)
{
    /* get pose from transform : map to baselink*/
    now_x = pose_ptr->pose.position.x;
    now_y = pose_ptr->pose.position.y;
    now_theta = tf2::getYaw(pose_ptr->pose.orientation);
    if (now_theta >= 3.14 || now_theta <= -3.14) now_theta = 3.1415926;
    ROS_INFO_STREAM("[Now pose]:" << now_x << "," << now_y << "," << now_theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "print_map_pose");
    ros::NodeHandle nh;
    ros::Subscriber mapPose_sub_ = nh.subscribe("/map_pose", 10, &mapPoseCallback);
    ros::spin();
}