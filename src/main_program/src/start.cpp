#include <ros/ros.h>
#include "wiringPi.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
​
#define gpio_pin 26
​
int main(int argc, char **argv)
{
    ros::init(argc, argv, "switch_test_state");
    ros::NodeHandle nh;
    wiringPiSetupGpio();
    pullUpDnControl(gpio_pin, PUD_UP);
​
    // ros::ServiceClient start_client = nh.serviceClient<std_srvs::Empty>("/startRunning");
    ros::Publisher start_pub = nh.advertise<std_msgs::Int32>("/start_running", 10);
    std_srvs::Empty srv_;
    std_msgs::Int32 topic_;
    int gpio_state = -1; // read gpio state
    int gpio_state_past = -1;
​
    while (ros::ok())
    {
        gpio_state_past = gpio_state;
        gpio_state = digitalRead(gpio_pin);
        ROS_INFO(gpio_state);
        // if (gpio_state == 1 && gpio_state_past == 0)
        // {
        //     ROS_INFO("***************");
        //     topic_.data = 1;
        //     start_pub.publish(topic_);
        //     start_client.call(srv_);
        //     // ROS_INFO("///////////////");
        //     break;
        // }
        // else
        // {
        //     topic_.data = 0;
        //     start_pub.publish(topic_);
        // }
​
        ros::spinOnce();
    }
}