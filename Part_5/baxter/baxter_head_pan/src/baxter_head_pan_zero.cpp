//utility to send head pan angle to zero
#include <ros/ros.h>
#include <baxter_core_msgs/HeadPanCommand.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_head_pan_zero"); 
    ros::NodeHandle n; 
    //create a publisher to send commands to Baxter's head pan
    ros::Publisher head_pan_pub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1);

    baxter_core_msgs::HeadPanCommand headPanCommand; //corresponding message type for head-pan control
    headPanCommand.target = 0.0; //set desired angle
    headPanCommand.enable_pan_request=1;
    ros::Rate timer(4);
    for (int i=0;i<4;i++) { //send this command multiple times before quitting
        //if node sends message and then quits immediately, message can get lost
        head_pan_pub.publish(headPanCommand);
        timer.sleep();
    }
    return 0;
}

