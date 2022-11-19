#include <ros/ros.h>
#include <main_program/starting.h>

#ifdef __aarch64__
#include <wiringPi.h>
#endif

#define start_pin 26    // BCM_PIN 26
#define level_2_pin 12  // BCM_PIN 12
#define level_3_pin 16  // BCM_PIN 16

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_caller");
    ros::NodeHandle nh;

#ifdef __aarch64__
    wiringPiSetupGpio();
    pullUpDnControl(start_pin, PUD_UP);
    pullUpDnControl(level_2_pin, PUD_UP);
    pullUpDnControl(level_3_pin, PUD_UP);

    ros::ServiceClient start_client = nh.serviceClient<main_program::starting>("/startRunning");
    main_program::starting start_srv;
    int start_state = -1; // read start state
    int start_state_past = -1;
    int level_2_state = -1; // read level 2 state
    int level_3_state = -1; // read level 3 state

    while (ros::ok())
    {
        start_state_past = start_state;
        start_state = digitalRead(start_pin);
        level_2_state = digitalRead(level_2_pin);
        level_3_state = digitalRead(level_3_pin);
        ROS_INFO_STREAM();
        if (start_state == 0 && start_state_past == 1){
            ROS_INFO_STREAM("[start_running !!!]");
            break;
        }
        if (level_2_state == 0){
            ROS_INFO_STREAM("[level]: " << "Set to level 2");
        }
        if (level_3_state == 0){
            ROS_INFO_STREAM("[level]: " << "Set to level 3");
        }
   
        ros::spinOnce();
    }

#endif
}