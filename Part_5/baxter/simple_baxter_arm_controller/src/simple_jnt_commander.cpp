#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <ros/init.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
    
using namespace std;

ros::Publisher joint_cmd_pub_right; //define this publisher global, so fnc cmd_pose_right() can use it

//a simple function to send joint-angle commands to joint controllers using a Baxter message:
void  cmd_pose_right(double qvec[]) {
    //member var right_cmd_ already has joint names populated; just need to update the joint-angle commands
    for (int i = 0; i < 7; i++) {
        right_cmd.command[i] = qvec[i];
    }
    joint_cmd_pub_right.publish(right_cmd); //send result to joint controllers
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rt_arm_pos_commander"); // name this node     
    ros::NodeHandle nh;
    baxter_core_msgs::JointCommand right_cmd;

    //point this publisher to Baxter's joint-command topic for right arm
    joint_cmd_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
    
    //initialize fields of joint command message:
    right_cmd.mode = 1; //specify position control

    // define the joint angles 0-6 to be right arm, from shoulder out to wrist;
    right_cmd.names.push_back("right_s0");
    right_cmd.names.push_back("right_s1");
    right_cmd.names.push_back("right_e0");
    right_cmd.names.push_back("right_e1");
    right_cmd.names.push_back("right_w0");
    right_cmd.names.push_back("right_w1");
    right_cmd.names.push_back("right_w2");

    // do push-backs to establish desired vector size with valid joint angles
    for (int i = 0; i < 7; i++) {
        right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
    }
    
    //do interesting stuff here:
    

}
