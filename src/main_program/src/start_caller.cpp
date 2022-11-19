#include <ros/ros.h>
#include <main_program/starting.h>

#ifdef __aarch64__
#include <wiringPi.h>
#endif

#define start_pin 26 // BCM_PIN 26

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_caller");
    ros::NodeHandle nh;

#ifdef __aarch64__
    wiringPiSetupGpio();
    pullUpDnControl(start_pin, PUD_UP);

    ros::ServiceClient start_client = nh.serviceClient<main_program::starting>("/startRunning");
    main_program::starting start_srv
    int start_state = -1; // read start state
    int start_state_past = -1;

    while (ros::ok())
    {
        start_state_past = start_state;
        start_state = digitalRead(start_pin);
        ROS_INFO_STREAM(start_state);
        if (start_state == 0 && start_state_past == 1)
        {
            ROS_INFO_STREAM("[start_state:]" << start_state);
            break;
        }
        // else
        // {
        //     topic_.data = 0;
        //     start_pub.publish(topic_);
        // }
        
        ros::spinOnce();
    }

#endif
}