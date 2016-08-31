// baxter_gripper:  a library to simplify baxter gripper I/O
// wsn, August, 2016
#ifndef BAXTER_GRIPPER_H
#define	BAXTER_GRIPPER_H
#include<ros/ros.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

class BaxterGripper {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gripper_subscriber_right_,gripper_subscriber_left_; //these will be set up within the class constructor, hiding these ugly details
    ros::Publisher  gripper_publisher_right_,gripper_publisher_left_;    
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void right_gripper_CB(const baxter_core_msgs::EndEffectorState& gripper_state);
    void left_gripper_CB(const baxter_core_msgs::EndEffectorState& gripper_state);    
    double gripper_pos_filter_val_,right_gripper_pos_,left_gripper_pos_;
    baxter_core_msgs::EndEffectorCommand gripper_cmd_open, gripper_cmd_close; 
 public:   
    void right_gripper_close(void) { gripper_publisher_right_.publish(gripper_cmd_close);};
    void left_gripper_close(void) { gripper_publisher_left_.publish(gripper_cmd_close);};  
    void right_gripper_open(void) { gripper_publisher_right_.publish(gripper_cmd_open);};
    void left_gripper_open(void) { gripper_publisher_left_.publish(gripper_cmd_open);};  
    double get_right_gripper_pos(void) { return right_gripper_pos_;};
    double get_left_gripper_pos(void) { return left_gripper_pos_;};    

    BaxterGripper(ros::NodeHandle* nodehandle);

};
#endif
