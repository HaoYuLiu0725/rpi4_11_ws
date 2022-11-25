#include <arm_move/arm_move.h>

using namespace std;
using namespace arm_move;

ArmMove::ArmMove(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
    timer_ = nh_.createTimer(ros::Duration(1.0), &ArmMove::timerCallback, this, false, false);
    initialize();
}

bool ArmMove::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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
    get_param_ok = nh_local_.param<double>("square2_x", p_square2_x, 0.0);          // [mm]
    get_param_ok = nh_local_.param<double>("square2_y", p_square2_y, 200.0);        // [mm]
    get_param_ok = nh_local_.param<double>("square2_z", p_square2_z, -15.0);        // [mm]
    get_param_ok = nh_local_.param<double>("touch_board_x", p_touch_board_x, 500.0);// [mm]
    get_param_ok = nh_local_.param<double>("touch_board_y", p_touch_board_y, 0.0);  // [mm]
    get_param_ok = nh_local_.param<double>("touch_board_z", p_touch_board_z, 10.0); // [mm]
    get_param_ok = nh_local_.param<double>("drop_offset", p_drop_offset_, 10.0);    // [mm]
    get_param_ok = nh_local_.param<double>("suck_offset", p_suck_offset_, -5.0);    // [mm]
    get_param_ok = nh_local_.param<double>("put_offset", p_put_offset_, -5.0);      // [mm]
    get_param_ok = nh_local_.param<double>("stack_offset", p_stack_offset_, 63.0);  // [mm]

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("mission_goal_topic", p_mission_target_topic_, "/mission_target");
    get_param_ok = nh_local_.param<string>("mission_status_topic", p_mission_status_topic_, "/mission_status");
    get_param_ok = nh_local_.param<string>("arm_goal_topic", p_arm_goal_topic_, "/arm_goal");
    get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");
    get_param_ok = nh_local_.param<string>("arm_status_topic", p_arm_status_topic_, "/arm_status");
    get_param_ok = nh_local_.param<string>("suck_status_topic", p_suck_status_topic_, "/suck_status");

    /* check param */
    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Arm Move]: " << "param set ok");
    }
    else
    {
        ROS_WARN_STREAM("[Arm Move]: " << "param set fail");
    }

    /* ros node param */
    timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            mission_target_sub_ = nh_.subscribe(p_mission_target_topic_, 10, &ArmMove::missionTargetCallback, this);
            mission_status_pub_ = nh_.advertise<std_msgs::Bool>(p_mission_status_topic_, 10);
            arm_goal_pub_ = nh_.advertise<geometry_msgs::Point>(p_arm_goal_topic_, 10);
            suck_pub_ = nh_.advertise<std_msgs::Bool>(p_suck_topic_, 10);
            arm_status_sub_ = nh_.subscribe(p_arm_status_topic_, 10, &ArmMove::armStatusCallback, this);
            suck_status_sub_ = nh_.subscribe(p_suck_status_topic_, 10, &ArmMove::suckStatusCallback, this);
            timer_.start();
        }
        else
        {
            mission_target_sub_.shutdown();
            mission_status_pub_.shutdown();
            arm_goal_pub_.shutdown();
            suck_pub_.shutdown();
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

    mission_status.data = true; // no mission
    mission_status_pub_.publish(mission_status);

    storage_1.x = p_storage1_x;
    storage_1.y = p_storage1_y;
    storage_1.z = p_storage1_z;
    storage_2.x = p_storage2_x;
    storage_2.y = p_storage2_y;
    storage_2.z = p_storage2_z;
    square_2.x = p_square2_x;
    square_2.y = p_square2_y;
    square_2.z = p_square2_z;
    touch_board.x = p_touch_board_x;
    touch_board.y = p_touch_board_y;
    touch_board.z = p_touch_board_z;

    pub_once = true;
    running = false;
    point_num = 0;
    more_suck_offset = 0;
    redo_count = 0;

    get_T = true;
    get_E = true;
    get_L = true;
    have_storage1 = false;
    have_storage2 = false;
    have_on_hand = false;
    block_stacked = 0;

    return true;
}

void ArmMove::missionTargetCallback(const arm_move::mission::ConstPtr& ptr)
{
    input_mission = *ptr;
    if(input_mission.type == 1){
        mission_state = mission_1;
        T_point = input_mission.T;
        E_point = input_mission.E;
        L_point = input_mission.L;
        point_num = 1;
        ROS_INFO_STREAM("[Arm Move]: Mission 1 received");
        ROS_INFO_STREAM("T_point" << T_point);
        ROS_INFO_STREAM("E_point" << E_point);
        ROS_INFO_STREAM("L_point" << L_point);
        if(T_point.x == 0 && T_point.y == 0 && T_point.z == 0) get_T = false;
        if(E_point.x == 0 && E_point.y == 0 && E_point.z == 0) get_E = false;
        if(L_point.x == 0 && L_point.y == 0 && L_point.z == 0) get_L = false;

        if(get_T) {goto_state = Goto_T_point; ROS_INFO_STREAM("[Arm Move]: Mission 1 started -> T");}
        else if(get_E) {goto_state = Goto_E_point; ROS_INFO_STREAM("[Arm Move]: Mission 1 started -> E");}
        else if(get_L) {goto_state = Goto_L_point; ROS_INFO_STREAM("[Arm Move]: Mission 1 started -> L");}
        else mission_state = no_mission;
    }
    else if(input_mission.type == 2){
        mission_state = mission_2;
        point_num = 1;
        ROS_INFO_STREAM("[Arm Move]: Mission 2 received");
        if(have_on_hand) {goto_state = Goto_square_2; ROS_INFO_STREAM("[Arm Move]: Mission 2 started -> square_2");}
        else if(have_storage2) {goto_state = Goto_storage_2; ROS_INFO_STREAM("[Arm Move]: Mission 2 started -> storage_2");}
        else if(have_storage1) {goto_state = Goto_storage_1; ROS_INFO_STREAM("[Arm Move]: Mission 2 started -> storage_1");}
        else mission_state = no_mission;
    }
    else if(input_mission.type == 3){
        mission_state = mission_3;
        point_num = 1;
        ROS_INFO_STREAM("[Arm Move]: Mission 3 received");
    }
    else{
        mission_state = no_mission;
        ROS_INFO_STREAM("[Arm Move]: ERROR Mission type !");
    }
    pub_once = true;
}

void ArmMove::armStatusCallback(const std_msgs::Bool::ConstPtr& ptr)  
{
    arm_status = *ptr;
    if(arm_status.data) running = false;
    else running = true;
}  

void ArmMove::suckStatusCallback(const std_msgs::Bool::ConstPtr& ptr)  
{
    suck_status = *ptr;
    ROS_INFO_STREAM("[Arm Move]: suck status : " << suck_status);
}

void ArmMove::timerCallback(const ros::TimerEvent& e)
{
    if (mission_state == no_mission && pub_once){
        publishMissionStatus(true); // mission done or no mission
        ROS_INFO_STREAM("[Arm Move]: Mission finished, wait for new mission!");
        pub_once = false;
    }
    else if (mission_state == mission_1) mission1();
    else if (mission_state == mission_2) mission2();
    else if (mission_state == mission_3) mission3();
}

/*----- Mission 1-----------------------------------------------------------------------------------*/
void ArmMove::mission1() /* In level 1, pick up T, E, L block in first square  */
{
    if(goto_state == Goto_T_point) goTo_T_Point();
    else if(goto_state == Goto_E_point) goTo_E_Point();
    else if(goto_state == Goto_L_point) goTo_L_Point();
    else if(goto_state == Goto_storage_1) goTo_Storage_1();        
    else if(goto_state == Goto_storage_2) goTo_Storage_2();
    else if(goto_state == Goto_square_2) goTo_Square_2();
}

void ArmMove::goTo_T_Point()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.x = T_point.x;
                output_point.y = T_point.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to T_point");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached T_point");
                publishSuck(true); // suction on
                publishArmGoal(T_point.x, T_point.y, T_point.z + p_suck_offset_ + more_suck_offset);
                ROS_INFO_STREAM("[Arm Move]: Go to T_point -> Z + suck");
                nextCase();
                break;
            case 3:
                if (!suck_status.data && redo_count < 3){   // suction faild
                    lastCase(-2.5);
                    break;
                }
                else{   // suction success
                    ROS_INFO_STREAM("[Arm Move]: Suction Success !");
                    more_suck_offset = 0;
                    redo_count = 0;
                }
                ROS_INFO_STREAM("[Arm Move]: Reached T_point -> Z + suck");
                get_T = false; //finished T_point
                check_Storage();
                break;
        }
    }
}

void ArmMove::goTo_E_Point()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.x = E_point.x;
                output_point.y = E_point.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to E_point");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached E_point");
                publishSuck(true); // suction on
                publishArmGoal(E_point.x, E_point.y, E_point.z + p_suck_offset_ + more_suck_offset);
                ROS_INFO_STREAM("[Arm Move]: Go to E_point -> Z + suck");
                nextCase();
                break;
            case 3:
                if (!suck_status.data && redo_count < 3){   // suction faild
                    lastCase(-2.5);
                    break;
                }
                else{   // suction success
                    ROS_INFO_STREAM("[Arm Move]: Suction Success !");
                    more_suck_offset = 0;
                    redo_count = 0;
                }
                ROS_INFO_STREAM("[Arm Move]: Reached E_point -> Z + suck");
                get_E = false; //finished T_point
                check_Storage();
                break;
        }
    }
}

void ArmMove::goTo_L_Point()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.x = L_point.x;
                output_point.y = L_point.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to L_point");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached L_point");
                publishSuck(true); // suction on
                publishArmGoal(L_point.x, L_point.y, L_point.z + p_suck_offset_ + more_suck_offset);
                ROS_INFO_STREAM("[Arm Move]: Go to L_point -> Z + suck");
                nextCase();
                break;
            case 3:
                if (!suck_status.data && redo_count < 3){   // suction faild
                    lastCase(-2.5);
                    break;
                }
                else{   // suction success
                    ROS_INFO_STREAM("[Arm Move]: Suction Success !");
                    more_suck_offset = 0;
                    redo_count = 0;
                }
                ROS_INFO_STREAM("[Arm Move]: Reached L_point -> Z + suck");
                get_L = false; //finished L_point
                check_Storage();
                break;
        }
    }
}

void ArmMove::goTo_Storage_1()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_1.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1 -> Z + drop");
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1");
                publishSuck(false); // suction off(release)
                ros::Duration(1).sleep();
                have_storage1 = true; //storage1 have block
                check_TEL_Point();
                break;
        }
    }
}

void ArmMove::goTo_Storage_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_2.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2 -> Z + drop");
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2");
                publishSuck(false); // suction off(release)
                ros::Duration(1).sleep();
                have_storage2 = true; //storage2 have block
                check_TEL_Point();
                break;
        }
    }
}

void ArmMove::goTo_Square_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.x = square_2.x;
                output_point.y = square_2.y;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2");
                publishArmGoal(square_2.x, square_2.y, square_2.z + 50);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2 -> wait_mission_2");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2 -> wait_mission_2");
                ROS_INFO_STREAM("[Arm Move]: Mission 1 finished");
                have_on_hand = true;
                finalCase();
                break; 
        }
    }
}

void ArmMove::check_TEL_Point() /* change state */
{
    if(get_T) {goto_state = Goto_T_point; point_num = 1;}
    else if(get_E) {goto_state = Goto_E_point; point_num = 1;}
    else if(get_L) {goto_state = Goto_L_point; point_num = 1;}
    /* mission finish*/
    else mission_state = no_mission;
}

void ArmMove::check_Storage() /* change state */ 
{
    if(!have_storage1) {goto_state = Goto_storage_1; point_num = 1;}
    else if(!have_storage2) {goto_state = Goto_storage_2; point_num = 1;}
    else {goto_state = Goto_square_2; point_num = 1;}
}

/*----- Mission 2-----------------------------------------------------------------------------------*/
void ArmMove::mission2() /* In level 2, put T, E, L block in second square  */
{
    if(goto_state == Goto_square_2) stack_Square_2();
    else if(goto_state == Goto_storage_2) stack_Storage_2();
    else if(goto_state == Goto_storage_1) stack_Storage_1();
    else mission_state = no_mission;
}

void ArmMove::stack_Square_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                publishArmGoal(square_2.x, square_2.y, square_2.z + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2 -> Z + put");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2 -> Z + put");
                publishSuck(false); // suction off(release)
                ros::Duration(1).sleep();
                have_on_hand = false; // block on hand stack complete
                block_stacked += 1;
                check_Stack();
                break;
        }
    }
}

void ArmMove::stack_Storage_2()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_2.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2 -> Z + drop");
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2");
                publishSuck(true); // suction on
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_suck_offset_ + more_suck_offset);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2 -> Z + suck");
                nextCase();
                break;
            case 4:
                if (!suck_status.data && redo_count < 3){   // suction faild
                    lastCase(-2.5);
                    break;
                }
                else{   // suction success
                    ROS_INFO_STREAM("[Arm Move]: Suction Success !");
                    more_suck_offset = 0;
                    redo_count = 0;
                }
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2 -> Z + suck"); //got block
                publishArmGoal(storage_2.x, storage_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_2 -> Z + drop"); // lift up to leave container
                nextCase();
                break;
            case 5:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_2 -> Z + drop");
                publishArmGoal(square_2.x, square_2.y, storage_2.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2");
                nextCase();
                break;
            case 6:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2");
                publishArmGoal(square_2.x, square_2.y, square_2.z + block_stacked * p_stack_offset_ + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2 -> Z + stack: " << block_stacked);
                nextCase();
                break;
            case 7:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2 -> Z + stack: " << block_stacked);
                publishSuck(false); // suction off(release)
                ros::Duration(1).sleep();
                have_storage2 = false; // block in storage2 stack complete
                block_stacked += 1;
                check_Stack();
                break;
        }
    }   
}

void ArmMove::stack_Storage_1()
{
    if(!running && point_num != 0){
        switch(point_num){
            case 1:
                output_point.z = storage_1.z + p_drop_offset_;
                arm_goal_pub_.publish(output_point);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1 -> Z + drop");
                nextCase();
                break;
            case 2:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1 -> Z + drop");
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1");
                nextCase();
                break;
            case 3:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1");
                publishSuck(true); // suction on
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_suck_offset_ + more_suck_offset);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1 -> Z + suck");
                nextCase();
                break;
            case 4:
                if (!suck_status.data && redo_count < 3){   // suction faild
                    lastCase(-2.5);
                    break;
                }
                else{   // suction success
                    ROS_INFO_STREAM("[Arm Move]: Suction Success !");
                    more_suck_offset = 0;
                    redo_count = 0;
                }
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1 -> Z + suck"); //got block
                publishArmGoal(storage_1.x, storage_1.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to storage_1 -> Z + drop"); // lift up to leave container
                nextCase();
                break;
            case 5:
                ROS_INFO_STREAM("[Arm Move]: Reached storage_1 -> Z + drop");
                publishArmGoal(square_2.x, square_2.y, storage_1.z + p_drop_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2");
                nextCase();
                break;
            case 6:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2");
                publishArmGoal(square_2.x, square_2.y, square_2.z + (block_stacked * p_stack_offset_) + p_put_offset_);
                ROS_INFO_STREAM("[Arm Move]: Go to square_2 -> Z + stack: " << block_stacked);
                nextCase();
                break;
            case 7:
                ROS_INFO_STREAM("[Arm Move]: Reached square_2 -> Z + stack: " << block_stacked);
                publishSuck(false); // suction off(release)
                ros::Duration(1).sleep();
                have_storage1 = false; // block in storage1 stack complete
                check_Stack();
                break;
        }
    } 
}
void ArmMove::backToInitArm()
{
    publishArmGoal(init_arm.x, init_arm.y, init_arm.z);
    finalCase();
}

void ArmMove::check_Stack() /* change state */ 
{
    if(have_on_hand) {goto_state = Goto_square_2; point_num = 1;}
    else if(have_storage2) {goto_state = Goto_storage_2; point_num = 1;}
    else if(have_storage1) {goto_state = Goto_storage_1; point_num = 1;}
    else {
        backToInitArm();
        mission_state = no_mission;
    }
}

/*----- Mission 3-----------------------------------------------------------------------------------*/
void ArmMove::mission3()
{
    arm_goal_pub_.publish(touch_board);
    mission_state = no_mission;
}

void ArmMove::publishArmGoal(double x, double y, double z)
{
    output_point.x = x;
    output_point.y = y;
    output_point.z = z;
    arm_goal_pub_.publish(output_point);
}

void ArmMove::publishSuck(bool state)
{
    suck.data = state;
    suck_pub_.publish(suck);
}

void ArmMove::publishMissionStatus(bool state)
{
    mission_status.data = state;
    mission_status_pub_.publish(mission_status);
}

void ArmMove::lastCase(double offset) 
{
    ROS_INFO_STREAM("[Arm Move]: Suction Failed !");
    point_num -= 1;
    running = false;
    more_suck_offset += offset;
    redo_count += 1;
}

void ArmMove::nextCase()
{
    point_num += 1;
    running = true;
}

void ArmMove::finalCase()
{
    point_num = 0;
    running = false;
    mission_state = no_mission;
    pub_once = true;
}