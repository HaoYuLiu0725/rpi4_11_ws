#include <arm_move/arm_move_project.h>

using namespace std;
using namespace arm_move_project;

ArmMoveProject::ArmMoveProject(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
    timer_ = nh_.createTimer(ros::Duration(1.0), &ArmMoveProject::timerCallback, this, false, false);
    params_srv_ = nh_.advertiseService("arm_move_reset", &ArmMoveProject::updateParams, this);
    initialize();
}

bool ArmMoveProject::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    /* get param */
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);

    get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 10);
    get_param_ok = nh_local_.param<double>("init_arm_x", p_init_arm_x, 128.0);      // [mm]
    get_param_ok = nh_local_.param<double>("init_arm_y", p_init_arm_y, 17.0);       // [mm]
    get_param_ok = nh_local_.param<double>("init_arm_z", p_init_arm_z, 10.0);       // [mm]
    get_param_ok = nh_local_.param<double>("storage1_x", p_storage1_x, 82.0);       // [mm]
    get_param_ok = nh_local_.param<double>("storage1_y", p_storage1_y, -99.0);      // [mm]
    get_param_ok = nh_local_.param<double>("storage1_z", p_storage1_z, 74.0);       // [mm]
    get_param_ok = nh_local_.param<double>("storage2_x", p_storage2_x, 82.0);       // [mm]
    get_param_ok = nh_local_.param<double>("storage2_y", p_storage2_y, -199.0);     // [mm]
    get_param_ok = nh_local_.param<double>("storage2_z", p_storage2_z, 74.0);       // [mm]
    get_param_ok = nh_local_.param<double>("wait_x", p_wait_x, 0.0);                // [mm]
    get_param_ok = nh_local_.param<double>("wait_y", p_wait_y, 150.0);              // [mm]
    get_param_ok = nh_local_.param<double>("wait_z", p_wait_z, 55.0);               // [mm]
    get_param_ok = nh_local_.param<double>("put_x", p_put_x, 0.0);                  // [mm]
    get_param_ok = nh_local_.param<double>("put_y", p_put_y, 150.0);                // [mm]
    get_param_ok = nh_local_.param<double>("put_z", p_put_z, -15.0);                // [mm]
    get_param_ok = nh_local_.param<double>("drop_offset", p_drop_offset_, 35.0);    // [mm]
    get_param_ok = nh_local_.param<double>("suck_offset", p_suck_offset_, -2.0);    // [mm]
    get_param_ok = nh_local_.param<double>("put_offset", p_put_offset_, 3.0);       // [mm]
    get_param_ok = nh_local_.param<double>("stack_offset", p_stack_offset_, 62.0);  // [mm]
    get_param_ok = nh_local_.param<double>("vibrate_time", p_vibrate_time_, 3.0);   // [s]

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("mission_goal_topic", p_mission_target_topic_, "/mission_target");
    get_param_ok = nh_local_.param<string>("mission_status_topic", p_mission_status_topic_, "/mission_status");
    get_param_ok = nh_local_.param<string>("arm_goal_topic", p_arm_goal_topic_, "/arm_goal");
    get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");
    get_param_ok = nh_local_.param<string>("vibrate_topic", p_vibrate_topic_, "/vibrate");
    get_param_ok = nh_local_.param<string>("arm_status_topic", p_arm_status_topic_, "/arm_status");
    get_param_ok = nh_local_.param<string>("suck_status_topic", p_suck_status_topic_, "/suck_status");

    /* check param */
    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Arm Move Project]: " << "param set ok");
    }
    else
    {
        ROS_WARN_STREAM("[Arm Move Project]: " << "param set fail");
    }

    /* ros node param */
    timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            mission_target_sub_ = nh_.subscribe(p_mission_target_topic_, 10, &ArmMoveProject::missionTargetCallback, this);
            mission_status_pub_ = nh_.advertise<std_msgs::Bool>(p_mission_status_topic_, 10);
            arm_goal_pub_ = nh_.advertise<geometry_msgs::Point>(p_arm_goal_topic_, 10);
            suck_pub_ = nh_.advertise<std_msgs::Bool>(p_suck_topic_, 10);
            vibrate_pub_ = nh_.advertise<std_msgs::Bool>(p_vibrate_topic_, 10);
            arm_status_sub_ = nh_.subscribe(p_arm_status_topic_, 10, &ArmMoveProject::armStatusCallback, this);
            suck_status_sub_ = nh_.subscribe(p_suck_status_topic_, 10, &ArmMoveProject::suckStatusCallback, this);
            timer_.start();
        }
        else
        {
            mission_target_sub_.shutdown();
            mission_status_pub_.shutdown();
            arm_goal_pub_.shutdown();
            suck_pub_.shutdown();
            vibrate_pub_.shutdown();
            arm_status_sub_.shutdown();
            suck_status_sub_.shutdown();
            timer_.stop();
        }
    }

    /* init state param */
    mission_state = no_mission;

    init_arm.x = p_init_arm_x;
    init_arm.y = p_init_arm_y;
    init_arm.z = p_init_arm_z;
    output_point = init_arm;
    arm_goal_pub_.publish(output_point); // initial arm pose

    suck.data = false; // no suction
    suck_pub_.publish(suck);

    vibrate.data = false; // no vibration
    vibrate_pub_.publish(vibrate);

    mission_status.data = true; // no mission
    mission_status_pub_.publish(mission_status);

    storage_1.x = p_storage1_x;
    storage_1.y = p_storage1_y;
    storage_1.z = p_storage1_z;
    storage_2.x = p_storage2_x;
    storage_2.y = p_storage2_y;
    storage_2.z = p_storage2_z;
    wait_point.x = p_wait_x;
    wait_point.y = p_wait_y;
    wait_point.z = p_wait_z;
    put_point.x = p_put_x;
    put_point.y = p_put_y;
    put_point.z = p_put_z;

    pub_once = true;
    running = false;
    point_num = 0;

    get_block_1 = true;
    get_block_2 = true;
    get_block_3 = true;
    have_storage1 = false;
    have_storage2 = false;
    have_wait = false;
    nead_stack_storage1 = false;
    nead_stack_storage2 = false;
    nead_stack_wait = false;
    have_on_hand = false;
    block_stacked = 0;

    return true;
}

void ArmMoveProject::missionTargetCallback(const arm_move::mission::ConstPtr& ptr)
{
    input_mission = *ptr;
    if(input_mission.type == 1){
        mission_state = mission_1;
        block_1 = input_mission.T;
        block_2 = input_mission.E;
        block_3 = input_mission.L;
        point_num = 1;
        ROS_INFO_STREAM("[Arm Move Project]: Mission 1 received");
        ROS_INFO_STREAM("block_1: " << block_1);
        ROS_INFO_STREAM("block_2: " << block_2);
        ROS_INFO_STREAM("block_3: " << block_3);
        if(block_1.x == 0 && block_1.y == 0 && block_1.z == 0) get_block_1 = false;
        if(block_2.x == 0 && block_2.y == 0 && block_2.z == 0) get_block_2 = false;
        if(block_3.x == 0 && block_3.y == 0 && block_3.z == 0) get_block_3 = false;

        if(get_block_1) {goto_state = Goto_block_1; ROS_INFO_STREAM("[Arm Move Project]: Mission 1 started -> block_1");}
        else if(get_block_2) {goto_state = Goto_block_2; ROS_INFO_STREAM("[Arm Move Project]: Mission 1 started -> block_2");}
        else if(get_block_3) {goto_state = Goto_block_3; ROS_INFO_STREAM("[Arm Move Project]: Mission 1 started -> block_3");}
        else mission_state = no_mission;
    }
    else if(input_mission.type == 2){
        mission_state = mission_2;
        nead_stack_storage1 = (input_mission.T.x == 0)? false : true;
        nead_stack_storage2 = (input_mission.E.x == 0)? false : true;
        nead_stack_wait = (input_mission.L.x == 0)? false : true;
        point_num = 1;
        ROS_INFO_STREAM("[Arm Move Project]: Mission 2 received");
        if(have_wait && nead_stack_wait) {goto_state = Goto_put_point; ROS_INFO_STREAM("[Arm Move Project]: Mission 2 started -> put_point");}
        else if(have_storage2 && nead_stack_storage2) {goto_state = Goto_storage_2; ROS_INFO_STREAM("[Arm Move Project]: Mission 2 started -> storage_2");}
        else if(have_storage1 && nead_stack_storage1) {goto_state = Goto_storage_1; ROS_INFO_STREAM("[Arm Move Project]: Mission 2 started -> storage_1");}
        else goto_state = Backto_init_arm;
        block_stacked = 0;
    }
    else{
        mission_state = no_mission;
        ROS_INFO_STREAM("[Arm Move Project]: " << input_mission.type << " is ERROR Mission type !");
    }
    pub_once = true;
}

void ArmMoveProject::armStatusCallback(const std_msgs::Bool::ConstPtr& ptr)  
{
    arm_status = *ptr;
    if(arm_status.data) running = false;
    else running = true;
}  

void ArmMoveProject::suckStatusCallback(const std_msgs::Bool::ConstPtr& ptr)  
{
    suck_status = *ptr;
    ROS_INFO_STREAM("[Arm Move Project]: suck status : " << suck_status);
}

void ArmMoveProject::timerCallback(const ros::TimerEvent& e)
{
    if (mission_state == no_mission && pub_once){
        publishMissionStatus(true); // mission done or no mission
        ROS_INFO_STREAM("[Arm Move Project]: Mission finished, wait for new mission!");
        pub_once = false;
    }
    else if (mission_state == mission_1) mission1();
    else if (mission_state == mission_2) mission2();
}

/*----- Mission 1-----------------------------------------------------------------------------------*/
void ArmMoveProject::mission1() /* pick up P, M, E block */
{
    if(goto_state == Goto_block_1) goTo_Block_1();
    else if(goto_state == Goto_block_2) goTo_Block_2();
    else if(goto_state == Goto_block_3) goTo_Block_3();
    else if(goto_state == Goto_storage_1) goTo_Storage_1();        
    else if(goto_state == Goto_storage_2) goTo_Storage_2();
    else if(goto_state == Goto_wait_point) goTo_Wait();
    else mission_state = no_mission;
}

void ArmMoveProject::goTo_Block_1()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                publishVibrate(true); // vibration ON
                output_point.x = block_1.x;
                output_point.y = block_1.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_1");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_1");
                publishSuck(true); // suction ON
                publishArmGoal(block_1.x, block_1.y, block_1.z + p_suck_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_1 -> Z + suck");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_1 -> Z + suck");
                if (!suck_status.data){ // get block FAILED
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 1 Failed !");
                    have_on_hand = false;   
                }
                else{   // get block SUCCESS
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 1 Success !");
                    have_on_hand = true; 
                }
                get_block_1 = false; //finished block_1
                check_Storage();
                break;
        }
    }
}

void ArmMoveProject::goTo_Block_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                publishVibrate(true); // vibration ON
                output_point.x = block_2.x;
                output_point.y = block_2.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_2");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_2");
                publishSuck(true); // suction ON
                publishArmGoal(block_2.x, block_2.y, block_2.z + p_suck_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_2 -> Z + suck");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_2 -> Z + suck");
                if (!suck_status.data){ // get block FAILED
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 2 Failed !");
                    have_on_hand = false;   
                }
                else{   // get block SUCCESS
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 2 Success !");
                    have_on_hand = true; 
                }
                get_block_2 = false; //finished block_2
                check_Storage();
                break;
        }
    }
}

void ArmMoveProject::goTo_Block_3()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                publishVibrate(true); // vibration ON
                output_point.x = block_3.x;
                output_point.y = block_3.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_3");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_3");
                publishSuck(true); // suction ON
                publishArmGoal(block_3.x, block_3.y, block_3.z + p_suck_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to block_3 -> Z + suck");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached block_3 -> Z + suck");
                if (!suck_status.data){ // get block FAILED
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 3 Failed !");
                    have_on_hand = false;   
                }
                else{   // get block SUCCESS
                    ROS_INFO_STREAM("[Arm Move Project]: Get Block 3 Success !");
                    have_on_hand = true; 
                }
                get_block_3 = false; //finished block_3
                check_Storage();
                break;
        }
    }
}

void ArmMoveProject::goTo_Storage_1()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_1.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1 -> Z + drop");
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1");
                publishSuck(false); // suction OFF(release)
                ros::Duration(1).sleep();
                have_storage1 = true; //storage1 have block
                check_Block();
                break;
        }
    }
}

void ArmMoveProject::goTo_Storage_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_2.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2 -> Z + drop");
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2");
                publishSuck(false); // suction OFF(release)
                ros::Duration(1).sleep();
                have_storage2 = true; //storage2 have block
                check_Block();
                break;
        }
    }
}

void ArmMoveProject::goTo_Wait()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = 55;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to safty Z");
                nextCase();
                break;
            case 2:
                output_point.x = wait_point.x;
                output_point.y = wait_point.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to wait_point");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached wait_point");
                ROS_INFO_STREAM("[Arm Move Project]: Mission 1 finished");
                have_wait = true;
                check_Block();
                finalCase();
                break; 
        }
    }
}

void ArmMoveProject::check_Block() /* change state */
{
    if(get_block_1) {goto_state = Goto_block_1; point_num = 1;}
    else if(get_block_2) {goto_state = Goto_block_2; point_num = 1;}
    else if(get_block_3) {goto_state = Goto_block_3; point_num = 1;}
    /* mission finish*/
    else{mission_state = no_mission; publishVibrate(false);} // vibration OFF
}

void ArmMoveProject::check_Storage() /* change state */ 
{
    if(have_on_hand){
        if(!have_storage1) {goto_state = Goto_storage_1; point_num = 1;}
        else if(!have_storage2) {goto_state = Goto_storage_2; point_num = 1;}
        else {goto_state = Goto_wait_point; point_num = 1;}
    }
    else{
        publishSuck(false); // suction OFF
        output_point.z = 80;
        arm_goal_pub_.publish(output_point);
        ROS_INFO_STREAM("[Arm Move Project]: Go to safty Z");
        ros::Duration(1).sleep();
        check_Block();
    }
}

/*----- Mission 2-----------------------------------------------------------------------------------*/
void ArmMoveProject::mission2() /* put P, M, E block in each P & M & E & stack squares  */
{
    if(goto_state == Goto_put_point) stack_Put();
    else if(goto_state == Goto_storage_2) stack_Storage_2();
    else if(goto_state == Goto_storage_1) stack_Storage_1();
    else if(goto_state == Backto_init_arm) backToInitArm();
    else mission_state = no_mission;
}

void ArmMoveProject::stack_Put()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.x = put_point.x;
                output_point.y = put_point.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point");
                publishArmGoal(put_point.x, put_point.y, put_point.z + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point -> Z + put");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point -> Z + put");
                publishSuck(false); // suction OFF(release)
                ros::Duration(1).sleep();
                have_wait = false; // block on hand(wait) stack complete
                block_stacked += 1;
                check_Stack();
                break;
        }
    }
}

void ArmMoveProject::stack_Storage_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_2.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2 -> Z + drop");
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2");
                publishSuck(true); // suction ON
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_suck_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2 -> Z + suck");
                nextCase();
                break;
            case 4:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2 -> Z + suck"); //got block
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_2 -> Z + drop"); // lift up to leave container
                nextCase();
                break;
            case 5:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_2 -> Z + drop");
                publishArmGoal(put_point.x, put_point.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point");
                nextCase();
                break;
            case 6:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point");
                publishArmGoal(put_point.x, put_point.y, put_point.z + (block_stacked * p_stack_offset_) + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point -> Z + stack: " << block_stacked);
                nextCase();
                break;
            case 7:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point -> Z + stack: " << block_stacked);
                publishSuck(false); // suction OFF(release)
                ros::Duration(1).sleep();
                have_storage2 = false; // block in storage2 stack complete
                block_stacked += 1;
                check_Stack();
                break;
        }
    }   
}

void ArmMoveProject::stack_Storage_1()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_1.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1 -> Z + drop");
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1");
                publishSuck(true); // suction ON
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_suck_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1 -> Z + suck");
                nextCase();
                break;
            case 4:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1 -> Z + suck"); //got block
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to storage_1 -> Z + drop"); // lift up to leave container
                nextCase();
                break;
            case 5:
                ROS_INFO_STREAM("[Arm Move Project]: Reached storage_1 -> Z + drop");
                publishArmGoal(put_point.x, put_point.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point");
                nextCase();
                break;
            case 6:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point");
                publishArmGoal(put_point.x, put_point.y, put_point.z + (block_stacked * p_stack_offset_) + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move Project]: Go to put_point -> Z + stack: " << block_stacked);
                nextCase();
                break;
            case 7:
                ROS_INFO_STREAM("[Arm Move Project]: Reached put_point -> Z + stack: " << block_stacked);
                publishSuck(false); // suction OFF(release)
                ros::Duration(1).sleep();
                have_storage1 = false; // block in storage1 stack complete
                block_stacked += 1;
                check_Stack();
                break;
        }
    } 
}
void ArmMoveProject::backToInitArm()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = 118;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Go to MAX_Z");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move Project]: Reached MAX_Z");
                output_point.x = init_arm.x;
                output_point.y = init_arm.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Back to init_arm");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move Project]: Reached init_arm");
                output_point.z = init_arm.z;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move Project]: Back to init_arm -> Z");
                nextCase();
                break;
            case 4:
                ROS_INFO_STREAM("[Arm Move Project]: Reached init_arm -> Z");
                ROS_INFO_STREAM("[Arm Move Project]: Mission 2 finished");
                finalCase();
                break;
        }
    }
}

void ArmMoveProject::check_Stack() /* change state */ 
{
    if(have_wait && nead_stack_wait) {goto_state = Goto_put_point; point_num = 1;}
    else if(have_storage2 && nead_stack_storage2) {goto_state = Goto_storage_2; point_num = 1;}
    else if(have_storage1 && nead_stack_storage1) {goto_state = Goto_storage_1; point_num = 1;}
    else {goto_state = Backto_init_arm; point_num = 1;}
}

/*--------------------------------------------------------------------------------------------------*/
void ArmMoveProject::publishArmGoal(double x, double y, double z)
{
    output_point.x = x;
    output_point.y = y;
    output_point.z = z;
    arm_goal_pub_.publish(output_point);
}

void ArmMoveProject::publishSuck(bool state)
{
    suck.data = state;
    suck_pub_.publish(suck);
}

void ArmMoveProject::publishVibrate(bool state)
{
    vibrate.data = state;
    vibrate_pub_.publish(vibrate);
}

void ArmMoveProject::publishMissionStatus(bool state)
{
    mission_status.data = state;
    mission_status_pub_.publish(mission_status);
}

void ArmMoveProject::nextCase()
{
    point_num += 1;
    running = true;
    ros::Duration(0.5).sleep();
}

void ArmMoveProject::finalCase()
{
    point_num = 0;
    running = false;
    mission_state = no_mission;
    pub_once = true;
}