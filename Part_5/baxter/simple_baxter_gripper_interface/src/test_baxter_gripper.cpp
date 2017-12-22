#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_baxter_gripper"); 
    ros::NodeHandle n; 
    ros::Publisher gripper_publisher_object = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1);
    ros::Rate naptime(1.0);
    
    //define and populate messages for gripper control:
    baxter_core_msgs::EndEffectorCommand gripper_cmd_open, gripper_cmd_close, configure;
    /*
    configure.id = 65538;
    configure.sequence = 1;
    configure.sender = "rsdk_gripper_keyboard_configure";
    configure.args = "{"holding_force": 30.0, "velocity": 50.0, "dead_zone": 5.0, "moving_force": 40.0}"
    */
    gripper_cmd_open.id = 65538;
    gripper_cmd_open.sequence = 2;
    gripper_cmd_open.command ="go";
    gripper_cmd_open.args = "{\"position\": 100.0}";
    gripper_cmd_open.sender = "rsdk_gripper_keyboard_go";
    gripper_cmd_close.command ="go";
    gripper_cmd_close.id = 65538;
    gripper_cmd_close.sequence = 3;
    gripper_cmd_close.args = "{\"position\": 0.0}";
    gripper_cmd_close.sender = "rsdk_gripper_keyboard_go";            
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        //close the right gripper fully
        gripper_publisher_object.publish(gripper_cmd_close); 
        naptime.sleep(); 
        gripper_publisher_object.publish(gripper_cmd_open); 
        naptime.sleep();         
    }
}

