//example to show how to set gripper_ID programmatically
#include <ros/ros.h>
#include <object_manipulation_properties/gripper_ID_codes.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_ID_setter"); 
    ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
    int gripper_id = GripperIdCodes::STICKY_FINGERS;
    nh.setParam("gripper_ID", gripper_id);
}

