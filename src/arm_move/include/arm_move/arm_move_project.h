#pragma once
/* All value is in [mm] */

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <arm_move/mission.h>

namespace arm_move_project
{
class ArmMoveProject
{
public:
    ArmMoveProject(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void missionTargetCallback(const arm_move::mission::ConstPtr& ptr); // from main program
    void armStatusCallback(const std_msgs::Bool::ConstPtr& ptr);        // from SCARA
    void suckStatusCallback(const std_msgs::Int16::ConstPtr& ptr);       // from SCARA
    void timerCallback(const ros::TimerEvent& e);

    void mission1();
    void goTo_Block_1();
    void goTo_Block_2();
    void goTo_Block_3();
    void goTo_Storage_1();
    void goTo_Storage_2();
    void goTo_Wait();
    void check_Block();
    void check_Storage();

    void mission2();
    void stack_Put();
    void stack_Storage_2();
    void stack_Storage_1();
    void check_Stack();
    void backToInitArm();
    
    void publishArmGoal(double x, double y, double z);
    void publishSuck(bool state);
    void publishVibrate(bool state);
    void publishMissionStatus(bool state);
    void nextCase();
    void finalCase();

    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer timer_;

    /* ros inter-node */
    ros::Subscriber mission_target_sub_;    // from main program
    ros::Publisher mission_status_pub_;     // to   main program
    ros::Publisher arm_goal_pub_;           // to   SCARA
    ros::Publisher suck_pub_;               // to   SCARA
    ros::Publisher vibrate_pub_;            // to   SCARA
    ros::Subscriber arm_status_sub_;        // from SCARA
    ros::Subscriber suck_status_sub_;       // from SCARA

    arm_move::mission input_mission;
    std_msgs::Bool mission_status; // true: mission done

    geometry_msgs::Point output_point;
    std_msgs::Bool suck;
    std_msgs::Bool vibrate;
    std_msgs::Bool arm_status;  // true: SCARA moving mission done ; false: SCARA can't reach current point
    std_msgs::Int16 suck_value; // preasure gauge value from SCARA

    ros::Time last_time_;
    ros::Duration timeout_;

    geometry_msgs::Point init_arm;
    geometry_msgs::Point block_1;
    geometry_msgs::Point block_2;
    geometry_msgs::Point block_3;
    geometry_msgs::Point storage_1;
    geometry_msgs::Point storage_2;
    geometry_msgs::Point wait_point;
    geometry_msgs::Point put_point;

    bool running; // true: arm is moving
    bool pub_once; // for mission_status_pub_ to pub only once
    int  point_num; // for switch case in each mission
    bool get_block_1;
    bool get_block_2;
    bool get_block_3;
    bool have_storage1;
    bool have_storage2;
    bool have_wait;
    bool have_on_hand;
    bool nead_stack_storage1;
    bool nead_stack_storage2;
    bool nead_stack_wait;
    int block_stacked;
    bool got_block;

    /* ros param */
    bool p_active_;

    double p_frequency_;
    double p_init_arm_x;
    double p_init_arm_y;
    double p_init_arm_z;
    double p_storage1_x;
    double p_storage1_y;
    double p_storage1_z;
    double p_storage2_x;
    double p_storage2_y;
    double p_storage2_z;
    double p_wait_x;
    double p_wait_y;
    double p_wait_z;
    double p_put_x;
    double p_put_y;
    double p_put_z;
    double p_drop_offset_;
    double p_suck_offset_;
    double p_put_offset_;
    double p_stack_offset_;
    double p_vibrate_time_;
    double p_preasure_threshold_;

    std::string p_mission_target_topic_;
    std::string p_mission_status_topic_;
    std::string p_arm_goal_topic_;
    std::string p_suck_topic_;
    std::string p_vibrate_topic_;
    std::string p_arm_status_topic_;
    std::string p_suck_status_topic_;

    /* state */
    enum Mission_State
    {
        no_mission,
        mission_1,
        mission_2
    }mission_state;

    enum Goto_State
    {
        Goto_block_1,
        Goto_block_2,
        Goto_block_3,
        Goto_storage_1,
        Goto_storage_2,
        Goto_wait_point,
        Goto_put_point,
        Backto_init_arm
    }goto_state;
};
}  // namespace arm_move
