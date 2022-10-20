#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <turtlesim/TeleportAbsolute.h>
#include <std_msgs/Bool.h>

geometry_msgs::Twist input_twist;

void twist_CallBack(const geometry_msgs::Twist::ConstPtr& ptr)
{
    input_twist = *ptr;
}
void pub_Callback(const ros::TimerEvent& event, ros::Publisher cmd_vel_pub, ros::Publisher base_speed_pub)
{
    cmd_vel_pub.publish(input_twist);
    base_speed_pub.publish(input_twist);
}

int main(int argc, char **argv)
{
    //node initialization
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &twist_CallBack);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher base_speed_pub = nh.advertise<geometry_msgs::Twist>("/base_speed", 10);
    
    ros::Timer pub_timer = nh.createTimer(ros::Duration(0.016), boost::bind(pub_Callback, _1, cmd_vel_pub, base_speed_pub));

    //call service to reset
    ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");
    std_srvs::Empty empty;
    reset.call(empty);
    ros::ServiceClient teleport = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute start_pos;
    start_pos.request.x = 0;
    start_pos.request.y = 0;
    start_pos.request.theta = 90 * M_PI / 180.0;
    teleport.call(start_pos);

    ros::spin();
}