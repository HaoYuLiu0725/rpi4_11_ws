#pragma once
/* All value is in [mm] */

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

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
    void missionGoalCallback(const geometry_msgs::Point::ConstPtr& ptr);  // from main program
    void missionCallback(const std_msgs::Int16::ConstPtr& ptr);           // from main program
    void armStatusCallback(const std_msgs::Bool::ConstPtr& ptr);          // from SCARA
    void timerCallback(const ros::TimerEvent& e);

//   void updateTwist();
//   void updatePose(const ros::TimerEvent& e);

    void publish();

    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer timer_;

    /* ros inter-node */
    ros::Subscriber mission_goal_sub_;   // from main program
    ros::Subscriber mission_sub_;         // from main program
    ros::Publisher mission_status_pub_;   // to   main program
    ros::Publisher arm_goal_pub_;         // to   SCARA
    ros::Publisher suck_pub_;             // to   SCARA
    ros::Subscriber arm_status_sub_;      // from SCARA

    geometry_msgs::Point mission_goal; /*****maybe vector*****/
    std_msgs::Int16 mission_num;
    std_msgs::Bool mission_status;

    geometry_msgs::Point arm_goal;
    std_msgs::Bool suck;
    std_msgs::Bool arm_status;

    ros::Time last_time_;
    ros::Duration timeout_;

    geometry_msgs::Point storage_1;
    geometry_msgs::Point storage_2;

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
    double p_drop_offset_;
    double p_suck_offset_;

    std::string p_mission_goal_topic_;
    std::string p_mission_topic_;
    std::string p_mission_status_topic_;
    std::string p_arm_goal_topic_;
    std::string p_suck_topic_;
    std::string p_arm_status_topic_;

    /* state */
    enum Mission_State
    {
        no_mission,
        mission_1,
        mission_2,
        mission_3
    }mission_state;
};
}  // namespace arm_move
