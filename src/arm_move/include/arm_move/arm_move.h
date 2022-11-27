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

namespace arm_move
{
class ArmMove
{
public:
    ArmMove(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void missionTargetCallback(const arm_move::mission::ConstPtr& ptr); // from main program
    void armStatusCallback(const std_msgs::Bool::ConstPtr& ptr);        // from SCARA
    void suckStatusCallback(const std_msgs::Bool::ConstPtr& ptr);       // from SCARA
    void timerCallback(const ros::TimerEvent& e);

    void mission1();
    void goTo_T_Point();
    void goTo_E_Point();
    void goTo_L_Point();
    void goTo_Storage_1();
    void goTo_Storage_2();
    void goTo_Square_2();
    void check_TEL_Point();
    void check_Storage();

    void mission2();
    void stack_Square_2();
    void stack_Storage_2();
    void stack_Storage_1();
    void check_Stack();
    void backToInitArm();

    void mission3();
    
    void publishArmGoal(double x, double y, double z);
    void publishSuck(bool state);
    void publishVibrate(bool state);
    void publishMissionStatus(bool state);
    void lastCase(double offset);
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
    std_msgs::Bool suck_status; // true: block suction successfully ; false: block release successfully

    ros::Time last_time_;
    ros::Duration timeout_;

    geometry_msgs::Point init_arm;
    geometry_msgs::Point T_point;
    geometry_msgs::Point E_point;
    geometry_msgs::Point L_point;
    geometry_msgs::Point storage_1;
    geometry_msgs::Point storage_2;
    geometry_msgs::Point square_2;
    geometry_msgs::Point touch_board;

    bool running; // true: arm is moving
    bool pub_once; // for mission_status_pub_ to pub only once
    int  point_num; // for switch case in each mission
    double more_suck_offset; // for suction failed on getting block, add more offset
    int  redo_count;
    bool get_T;
    bool get_E;
    bool get_L;
    bool have_storage1;
    bool have_storage2;
    bool have_on_hand;
    int block_stacked;

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
    double p_square2_x;
    double p_square2_y;
    double p_square2_z;
    double p_touch_board_x;
    double p_touch_board_y;
    double p_touch_board_z;
    double p_drop_offset_;
    double p_suck_offset_;
    double p_put_offset_;
    double p_stack_offset_;
    double p_vibrate_time_;

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
        mission_2,
        mission_3
    }mission_state;

    enum Goto_State
    {
        Goto_T_point,
        Goto_E_point,
        Goto_L_point,
        Goto_storage_1,
        Goto_storage_2,
        Goto_square_2,
        Backto_init_arm
    }goto_state;
};
}  // namespace arm_move
