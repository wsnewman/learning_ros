#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_publisher"); // name of this node will be "minimal_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher gripper_publisher_object = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1);

    
    baxter_core_msgs::EndEffectorCommand gripper_cmd; 
    gripper_cmd.command ="go";
    gripper_cmd.args = "{'position': 0.0}'";
    gripper_cmd.sender = "gripper_publisher";
    ros::Rate naptime(1.0);
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        //gripper_cmd.args = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
        gripper_cmd.args = "{'position': 0.0}'"; //close the right gripper fully
        gripper_publisher_object.publish(gripper_cmd); 
        naptime.sleep(); 
        gripper_cmd.args = "{'position': 100.0}'"; //open the right gripper fully
        gripper_publisher_object.publish(gripper_cmd); 
        naptime.sleep();         
    }
}

