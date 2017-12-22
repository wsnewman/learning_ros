// baxter_gripper:   a library to simplify gripper I/O
// wsn, August, 2016

#include<ros/ros.h>
#include<simple_baxter_gripper_interface/simple_baxter_gripper_interface.h>


BaxterGripper::BaxterGripper(ros::NodeHandle* nodehandle): nh_(*nodehandle) {
    gripper_cmd_open.id = 65538;
    gripper_cmd_open.command ="go";
    //gripper_cmd_open.args = "{'position': 100.0}'"; //oops
    gripper_cmd_open.args = "{\"position\": 100.0}";
    gripper_cmd_open.sender = "gripper_publisher";
    gripper_cmd_open.sequence = 2;
    
    gripper_cmd_close.id = 65538;
    gripper_cmd_close.command ="go";
    //gripper_cmd_close.args = "{'position': 0.0}'"; //oops
    gripper_cmd_close.args = "{\"position\": 0.0}";
    gripper_cmd_close.sender = "gripper_publisher"; 
    gripper_cmd_close.sequence = 3;
    
    gripper_pos_filter_val_ = 0.2;
    right_gripper_pos_ = -0.2;
    left_gripper_pos_ = -10.2;
    initializeSubscribers(); 
    initializePublishers();
}
void BaxterGripper::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    gripper_subscriber_right_ = nh_.subscribe("/robot/end_effector/right_gripper/state", 1, &BaxterGripper::right_gripper_CB,this); 
    gripper_subscriber_left_ = nh_.subscribe("/robot/end_effector/left_gripper/state", 1, &BaxterGripper::left_gripper_CB,this);     
}

void BaxterGripper::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    gripper_publisher_right_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1, true); 
    gripper_publisher_left_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1, true); 
}
void BaxterGripper::right_gripper_CB(const baxter_core_msgs::EndEffectorState& gripper_state) {
     //low-pass filter the gripper position for more reliable threshold tests
    right_gripper_pos_ = (1.0- gripper_pos_filter_val_)*right_gripper_pos_ + gripper_pos_filter_val_*gripper_state.position; 
} 
void BaxterGripper::left_gripper_CB(const baxter_core_msgs::EndEffectorState& gripper_state) {
     //low-pass filter the gripper position for more reliable threshold tests
    left_gripper_pos_ = (1.0- gripper_pos_filter_val_)*left_gripper_pos_ + gripper_pos_filter_val_*gripper_state.position; 
} 
