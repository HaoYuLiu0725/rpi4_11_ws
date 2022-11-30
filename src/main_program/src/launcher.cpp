#include <ros/ros.h>

#ifdef __aarch64__
#include <wiringPi.h>
#endif

#define launch_pin  20    // BCM_PIN 20

int main(int argc, char **argv)
{
    ros::init(argc, argv, "launcher");
    ros::NodeHandle nh;

    int launch_state = -1; // read launch state
    int launch_state_past = -1;
    bool isLaunched = false;

#ifdef __aarch64__
    wiringPiSetupGpio();
    pullUpDnControl(launch_pin, PUD_UP);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        launch_state_past = launch_state;
        launch_state = digitalRead(launch_pin);
        if(!isLaunched) ROS_INFO_STREAM("[Launch Waiting...] "<< launch_state);

        if (launch_state == 0 && launch_state_past == 1 && isLaunched == false){
            system("roslaunch main_program tel2022_rpi.launch");
            isLaunched = true;
        }
        if (launch_state == 1 && launch_state_past == 0 && isLaunched == true){
            system("rosnode kill /arm_serial_node");
            system("rosnode kill /base_serial_node");
            isLaunched = false;
        }
        rate.sleep();
    }
#endif
}