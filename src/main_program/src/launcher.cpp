#include <ros/ros.h>

#ifdef __aarch64__
#include <wiringPi.h>
#endif

#define launch_pin  26    // BCM_PIN 26
#define kill_pin    16    // BCM_PIN 16

int main(int argc, char **argv)
{
    ros::init(argc, argv, "launcher");
    ros::NodeHandle nh;

    int launch_state = -1; // read launch state
    int launch_state_past = -1;
    int kill_state = -1; // read level 2 state
    int kill_state_past = -1;

#ifdef __aarch64__
    wiringPiSetupGpio();
    pullUpDnControl(launch_pin, PUD_UP);
    pullUpDnControl(kill_pin, PUD_UP);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        launch_state_past = launch_state;
        kill_state_past = kill_state;
        launch_state = digitalRead(launch_pin);
        kill_state = digitalRead(kill_pin);
        ROS_INFO_STREAM("[Waiting...] "<< launch_state << ", " << kill_state);
        if (launch_state == 0 && launch_state_past == 1){
            system("roslaunch main_program tel2022_rpi.launch");
        }
        if (kill_state == 0 && kill_state_past == 1){
            system("rosnode kill /arm_serial_node");
            system("rosnode kill /base_serial_node");
        }
        rate.sleep();
    }
#endif
}