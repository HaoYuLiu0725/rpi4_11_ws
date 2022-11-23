#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

using namespace std;
#define SATURATION(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class RemoteControl
{
public:
    RemoteControl(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0), &RemoteControl::timerCallback, this, false, false);
        initialize();
    }

private:
    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer timer_;

    /* ros inter-node */
    ros::Subscriber joy_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher point_pub_;
    ros::Publisher suck_pub_;
    ros::Subscriber arm_status_sub_;        // from SCARA

    sensor_msgs::Joy input_joy_;
    geometry_msgs::Twist output_twist_;
    geometry_msgs::Point output_point_;
    std_msgs::Bool output_suck_;
    std_msgs::Bool arm_status;  // true: SCARA moving mission done ; false: SCARA can't reach current point

    ros::Time last_time_;
    ros::Duration timeout_;

    geometry_msgs::Point init_arm;
    geometry_msgs::Point storage_1;
    geometry_msgs::Point storage_2;
    geometry_msgs::Point putSquare;

    bool doMission;
    bool running; // true: arm is moving
    int8_t point_num; // for switch case in each mission
    double safe_z_point; // safe z point that won't collide while moving, equal to p_storage1_z and p_storage2_z

    /* ros param */
    bool p_active_;

    double p_frequency_;
    double p_MAX_linear_speed_;     // [m/s]
    double p_MAX_angular_speed_;    // [rad/s]
    double p_arm_MAX_XYspeed_;      // [mm/s]
    double p_arm_MAX_Zspeed_;       // [mm/s]
    double p_X_max_;                // [mm]
    double p_X_min_;                // [mm]
    double p_Y_max_;                // [mm]
    double p_Y_min_;                // [mm]
    double p_Z_max_;                // [mm]
    double p_Z_min_;                // [mm]
    double p_init_arm_x;
    double p_init_arm_y;
    double p_init_arm_z;
    double p_storage1_x;
    double p_storage1_y;
    double p_storage1_z;
    double p_storage2_x;
    double p_storage2_y;
    double p_storage2_z;
    double p_putSquare_x;
    double p_putSquare_y;
    double p_putSquare_z;

    std::string p_joy_topic_;
    std::string p_twist_topic_;
    std::string p_point_topic_;
    std::string p_suck_topic_;
    std::string p_arm_status_topic_;

    enum Mission_State
    {
        no_mission,
        goto_init_arm,
        goto_storage1,
        goto_storage2,
        goto_putSquare
    }mission_state;

    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        bool get_param_ok = true;
        bool prev_active = p_active_;

        /* get param */
        get_param_ok = nh_local_.param<bool>("active", p_active_, true);

        get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 50);
        get_param_ok = nh_local_.param<double>("MAX_linear_speed", p_MAX_linear_speed_, 0.8);   // [m/s]
        get_param_ok = nh_local_.param<double>("MAX_angular_speed", p_MAX_angular_speed_, 1.0); // [rad/s]
        get_param_ok = nh_local_.param<double>("arm_MAX_XYspeed", p_arm_MAX_XYspeed_, 100.0);   // [mm/s]
        get_param_ok = nh_local_.param<double>("arm_MAX_Zspeed", p_arm_MAX_Zspeed_, 100.0);     // [mm/s]
        get_param_ok = nh_local_.param<double>("X_max", p_X_max_, 516.0);                       // [mm]
        get_param_ok = nh_local_.param<double>("X_min", p_X_min_, -281.0);                      // [mm]
        get_param_ok = nh_local_.param<double>("Y_max", p_Y_max_, 516.0);                       // [mm]
        get_param_ok = nh_local_.param<double>("Y_min", p_Y_min_, -516.0);                      // [mm]
        get_param_ok = nh_local_.param<double>("Z_max", p_Z_max_, 119.0);                       // [mm]
        get_param_ok = nh_local_.param<double>("Z_min", p_Z_min_, -58.0);                       // [mm]
        get_param_ok = nh_local_.param<double>("init_arm_x", p_init_arm_x, 128.0);              // [mm]
        get_param_ok = nh_local_.param<double>("init_arm_y", p_init_arm_y, 17.0);               // [mm]
        get_param_ok = nh_local_.param<double>("init_arm_z", p_init_arm_z, 10.0);               // [mm]
        get_param_ok = nh_local_.param<double>("storage1_x", p_storage1_x, 82.0);               // [mm]
        get_param_ok = nh_local_.param<double>("storage1_y", p_storage1_y, -99.0);              // [mm]
        get_param_ok = nh_local_.param<double>("storage1_z", p_storage1_z, 74.0);               // [mm]
        get_param_ok = nh_local_.param<double>("storage2_x", p_storage2_x, 82.0);               // [mm]
        get_param_ok = nh_local_.param<double>("storage2_y", p_storage2_y, -199.0);             // [mm]
        get_param_ok = nh_local_.param<double>("storage2_z", p_storage2_z, 74.0);               // [mm]
        get_param_ok = nh_local_.param<double>("putSquare_x", p_putSquare_x, 0.0);              // [mm]
        get_param_ok = nh_local_.param<double>("putSquare_y", p_putSquare_y, 200.0);            // [mm]
        get_param_ok = nh_local_.param<double>("putSquare_z", p_putSquare_z, 50.0);            // [mm]

        double timeout;
        get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
        timeout_.fromSec(timeout);

        get_param_ok = nh_local_.param<string>("joy_topic", p_joy_topic_, "/ds5_joy");
        get_param_ok = nh_local_.param<string>("twist_topic", p_twist_topic_, "/cmd_vel");
        get_param_ok = nh_local_.param<string>("point_topic", p_point_topic_, "/arm_goal");
        get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");
        get_param_ok = nh_local_.param<string>("arm_status_topic", p_arm_status_topic_, "/arm_status");

        /* check param */
        if (get_param_ok){
            ROS_INFO_STREAM("[UWB Remote Control]: " << "param set ok");
        }
        else{
            ROS_WARN_STREAM("[UWB Remote Control]: " << "param set fail");
        }

        /* ros node param */
        timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

        if (p_active_ != prev_active)
        {
            if (p_active_)
            {
                joy_sub_ = nh_.subscribe(p_joy_topic_, 10, &RemoteControl::joyCallback, this);
                twist_pub_ = nh_.advertise<geometry_msgs::Twist>(p_twist_topic_, 10);
                point_pub_ = nh_.advertise<geometry_msgs::Point>(p_point_topic_, 10);
                suck_pub_ = nh_.advertise<std_msgs::Bool>(p_suck_topic_, 10);
                arm_status_sub_ = nh_.subscribe(p_arm_status_topic_, 10, &RemoteControl::armStatusCallback, this);
                timer_.start();
            }
            else
            {
                joy_sub_.shutdown();
                twist_pub_.shutdown();
                point_pub_.shutdown();
                suck_pub_.shutdown();
                arm_status_sub_.shutdown();
                timer_.stop();
            }
        }

        /* init state param */
        for(int i = 0; i < 6; i++){
            input_joy_.axes.push_back(0);
        }
        for(int i = 0; i < 18; i++){
            input_joy_.buttons.push_back(0);
        }
        output_twist_.linear.x = 0.0;
        output_twist_.linear.y = 0.0;
        output_twist_.angular.z = 0.0;

        init_arm.x = p_init_arm_x;
        init_arm.y = p_init_arm_y;
        init_arm.z = p_init_arm_z;
        output_point_ = init_arm;

        storage_1.x = p_storage1_x;
        storage_1.y = p_storage1_y;
        storage_1.z = p_storage1_z;
        storage_2.x = p_storage2_x;
        storage_2.y = p_storage2_y;
        storage_2.z = p_storage2_z;
        putSquare.x = p_putSquare_x;
        putSquare.y = p_putSquare_y;
        putSquare.z = p_putSquare_z;

        safe_z_point = p_storage1_z;
        output_suck_.data = false;

        doMission = false;
        running = false;
        point_num = 0;

        publish();

        return true;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& ptr)
    {
        input_joy_ = *ptr;
        last_time_ = ros::Time::now();
    }
    
    void armStatusCallback(const std_msgs::Bool::ConstPtr& ptr)  
    {
        arm_status = *ptr;
        if(arm_status.data) running = false;
        else running = true;
    }  

    void timerCallback(const ros::TimerEvent& e)
    {
        if(ros::Time::now().toSec() - last_time_.toSec() > timeout_.toSec()){
            return;
        }

        updateTwist();
        updatePoint(e);
        updateBool();

        if(!doMission || mission_state == no_mission){
            publish();
        }
        else{
            /* misson state machine*/
            if(mission_state == goto_init_arm){
                GoTo_init_arm();
            }
            else if(mission_state == goto_storage1){
                GoTo_storage1();
            }
            else if(mission_state == goto_storage2){
                GoTo_storage2();
            }
            else if(mission_state == goto_putSquare){
                GoTo_putSquare();
            }
        }
    }

    void updateTwist()
    {
        // Left stick, upward (-1.0 -> 1.0, default ~0.0)
        output_twist_.linear.x = input_joy_.axes[1] * p_MAX_linear_speed_;
        // Left stick, leftward (-1.0 -> 1.0, default ~0.0)
        output_twist_.linear.y = input_joy_.axes[0] * p_MAX_linear_speed_;
        // (L2 - R2), pushDown (0.0 -> 1.0, default = 0)
        output_twist_.angular.z = (input_joy_.axes[4] - input_joy_.axes[5]) * p_MAX_angular_speed_;
    }

    void updatePoint(const ros::TimerEvent& e)
    {
        double dt = (e.current_expected - e.last_expected).toSec();

        // Right stick, leftward (-1.0 -> 1.0, default ~0.0)
        double dx = - input_joy_.axes[2] * p_arm_MAX_XYspeed_ *  dt;
        // Right stick, upward (-1.0 -> 1.0, default ~0.0)
        double dy = input_joy_.axes[3] * p_arm_MAX_XYspeed_ * dt;
        // (button up - button down)
        double dz = (input_joy_.buttons[13] - input_joy_.buttons[14]) * p_arm_MAX_Zspeed_ * dt;

        output_point_.x += dx;
        output_point_.y += dy;
        output_point_.z += dz;

        output_point_.x = SATURATION(output_point_.x, p_X_min_, p_X_max_);
        output_point_.y = SATURATION(output_point_.y, p_Y_min_, p_Y_max_);
        output_point_.z = SATURATION(output_point_.z, p_Z_min_, p_Z_max_);

    }

    void updateBool()
    {
        /* button circle */
        if(input_joy_.buttons[1]){  
            output_suck_.data = true;
        }  

        /* button cross */     
        if(input_joy_.buttons[0]){  
            output_suck_.data = false;
        }

        /* button options */
        if(input_joy_.buttons[12]){ 
            doMission = true;
            mission_state = goto_init_arm;
            point_num = 1;
            ROS_INFO_STREAM("[Arm Move]/[Get Mission]: GoTo_init_arm");
        }

        /* rear right 1 (R1) */
        if(input_joy_.buttons[5]){
            doMission = true;
            mission_state = goto_storage1;
            point_num = 1;
            ROS_INFO_STREAM("[Arm Move]/[Get Mission]: GoTo_storage1");
        }

        /* rear left 1 (L1) */
        if(input_joy_.buttons[4]){
            doMission = true;
            mission_state = goto_storage2;
            point_num = 1;
            ROS_INFO_STREAM("[Arm Move]/[Get Mission]: GoTo_storage2");
        }

        /* button square */
        if(input_joy_.buttons[3]){
            doMission = true;
            mission_state = goto_putSquare;
            point_num = 1;
            ROS_INFO_STREAM("[Arm Move]/[Get Mission]: GoTo_putSquare");
        }

        /*button share*/
        if(input_joy_.buttons[11]){ 
            // abort any mission !!!
            ROS_INFO_STREAM("[Arm Move]/[Mission]: Abort mission !!!");
            doMission = false;
            finalCase();
        }
    }

    void publish()
    {
        /* twist */
        twist_pub_.publish(output_twist_);

        /* point */
        point_pub_.publish(output_point_);

        /* suck */
        suck_pub_.publish(output_suck_);
    }

    void publishArmGoal(double x, double y, double z)
    {
        output_point_.x = x;
        output_point_.y = y;
        output_point_.z = z;
        point_pub_.publish(output_point_);
    }

    void GoTo_init_arm()
    {
        if(!running && point_num != 0){
            switch(point_num){
                case 1:
                    ROS_INFO_STREAM("[Arm Move]/[Mission]: GoTo_init_arm");
                    output_point_.z = safe_z_point;
                    point_pub_.publish(output_point_); // go to safe_z_point
                    ROS_INFO_STREAM("[Arm Move]: go to safe_z_point");
                    nextCase();
                    break;
                case 2:
                    ROS_INFO_STREAM("[Arm Move]: reached safe_z_point !");
                    publishArmGoal(init_arm.x, init_arm.y, safe_z_point);
                    ROS_INFO_STREAM("[Arm Move]: go to init_arm");
                    nextCase();
                    break;
                case 3:
                    ROS_INFO_STREAM("[Arm Move]: reached init_arm !");
                    publishArmGoal(init_arm.x, init_arm.y, init_arm.z);
                    ROS_INFO_STREAM("[Arm Move]: go to init_arm -> Z");
                    nextCase();
                    break;
                case 4:
                    ROS_INFO_STREAM("[Arm Move]: reached init_arm -> Z");
                    ROS_INFO_STREAM("[Arm Move]/[Mission Finish]: GoTo_init_arm");
                    finalCase();
                    break;
            }
        }
    }

    void GoTo_storage1()
    {
        if(!running && point_num != 0){
            switch(point_num){
                case 1:
                    ROS_INFO_STREAM("[Arm Move]/[Mission]: GoTo_storage1");
                    output_point_.z = safe_z_point;
                    point_pub_.publish(output_point_); // go to safe_z_point
                    ROS_INFO_STREAM("[Arm Move]: go to safe_z_point");
                    nextCase();
                    break;
                case 2:
                    ROS_INFO_STREAM("[Arm Move]: reached safe_z_point !");
                    publishArmGoal(storage_1.x, storage_1.y, safe_z_point);
                    ROS_INFO_STREAM("[Arm Move]: go to storage1");
                    nextCase();
                    break;
                case 3:
                    ROS_INFO_STREAM("[Arm Move]: reached storage1 !");
                    publishArmGoal(storage_1.x, storage_1.y, storage_1.z);
                    ROS_INFO_STREAM("[Arm Move]: go to storage1 -> Z");
                    nextCase();
                    break;
                case 4:
                    ROS_INFO_STREAM("[Arm Move]: reached storage1 -> Z");
                    ROS_INFO_STREAM("[Arm Move]/[Mission Finish]: GoTo_storage1");
                    finalCase();
                    break;
            }
        }
    }

    void GoTo_storage2()
    {
        if(!running && point_num != 0){
            switch(point_num){
                case 1:
                    ROS_INFO_STREAM("[Arm Move]/[Mission]: GoTo_storage2");
                    output_point_.z = safe_z_point;
                    point_pub_.publish(output_point_); // go to safe_z_point
                    ROS_INFO_STREAM("[Arm Move]: go to safe_z_point");
                    nextCase();
                    break;
                case 2:
                    ROS_INFO_STREAM("[Arm Move]: reached safe_z_point !");
                    publishArmGoal(storage_2.x, storage_2.y, safe_z_point);
                    ROS_INFO_STREAM("[Arm Move]: go to storage2");
                    nextCase();
                    break;
                case 3:
                    ROS_INFO_STREAM("[Arm Move]: reached storage2 !");
                    publishArmGoal(storage_2.x, storage_2.y, storage_2.z);
                    ROS_INFO_STREAM("[Arm Move]: go to storage2 -> Z");
                    nextCase();
                    break;
                case 4:
                    ROS_INFO_STREAM("[Arm Move]: reached storage2 -> Z");
                    ROS_INFO_STREAM("[Arm Move]/[Mission Finish]: GoTo_storage2");
                    finalCase();
                    break;
            }
        }
    }

    void GoTo_putSquare()
    {
        if(!running && point_num != 0){
            switch(point_num){
                case 1:
                    ROS_INFO_STREAM("[Arm Move]/[Mission]: GoTo_putSquare");
                    output_point_.z = safe_z_point;
                    point_pub_.publish(output_point_); // go to safe_z_point
                    ROS_INFO_STREAM("[Arm Move]: go to safe_z_point");
                    nextCase();
                    break;
                case 2:
                    ROS_INFO_STREAM("[Arm Move]: reached safe_z_point !");
                    publishArmGoal(putSquare.x, putSquare.y, safe_z_point);
                    ROS_INFO_STREAM("[Arm Move]: go to putSquare");
                    nextCase();
                    break;
                case 3:
                    ROS_INFO_STREAM("[Arm Move]: reached putSquare !");
                    publishArmGoal(putSquare.x, putSquare.y, putSquare.z);
                    ROS_INFO_STREAM("[Arm Move]: go to putSquare -> Z");
                    nextCase();
                    break;
                case 4:
                    ROS_INFO_STREAM("[Arm Move]: reached putSquare -> Z");
                    ROS_INFO_STREAM("[Arm Move]/[Mission Finish]: GoTo_putSquare");
                    finalCase();
                    break;
            }
        }
    }

    void nextCase()
    {
        point_num += 1;
        running = true;
    }

    void finalCase()
    {
        point_num = 0;
        running = false;
        mission_state = no_mission;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uwb_remote_control");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[UWB Remote Control]: Initializing node");
    RemoteControl remote_control(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[UWB Remote Control]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[UWB Remote Control]: Unexpected error");
  }

  return 0;
}