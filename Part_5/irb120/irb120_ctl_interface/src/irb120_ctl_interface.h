// irb120_ctl_interface.h header file //
// wsn; Dec, 2017
// include this file in "irb120_ctl_interface.cpp"

// this is a node to take the place of ROS-industrial's "motion_download_interface"


// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef IRB120_ROBOT_INTERFACE_H_
#define IRB120_ROBOT_INTERFACE_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

//#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

const double UPDATE_RATE=10.0; // define the update rate for sending trajectory points 

// define a class, including a constructor, member variables and member functions
class Irb120RobotInterface
{
public:
    Irb120RobotInterface(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    ros::Duration dt_move_;
    void sendTrajPointCmd(); // send the current trajectory point, if available, and advance the pointer
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber sub_joint_trajectory_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber sub_joint_state_;
    ros::Publisher  joint_command_publisher_;
    ros::Publisher  joint1_command_publisher_;
    ros::Publisher  joint2_command_publisher_;
    ros::Publisher  joint3_command_publisher_;
    ros::Publisher  joint4_command_publisher_;
    ros::Publisher  joint5_command_publisher_;
    ros::Publisher  joint6_command_publisher_;    
    trajectory_msgs::JointTrajectory new_trajectory_;
    trajectory_msgs::JointTrajectoryPoint current_trajectory_point_;
    sensor_msgs::JointState new_joint_state_;
    int npts_traj_;
    int ith_point_;
    int njoints_;
    std_msgs::Float64 pos_cmd_;

    ros::Duration current_t_from_start_;
    ros::Duration prev_t_from_start_;        
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    
    void jointTrajectoryCB(const trajectory_msgs::JointTrajectory &msg);
    void jointStateCB(const sensor_msgs::JointState & msg);

    void print_point(trajectory_msgs::JointTrajectoryPoint);
    void print_joint_names(trajectory_msgs::JointTrajectory traj);
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for example service
    //bool serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
