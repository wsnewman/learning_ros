// wsn test pgm to command joint values to Baxter
// accept keyboard commands
#ifndef ARM7DOF_TRAJ_STREAMER_H_
#define ARM7DOF_TRAJ_STREAMER_H_

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <ros/init.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <std_srvs/Trigger.h>


typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
using namespace std;

const double q0dotmax = 0.17;
const double q1dotmax = 0.17;
const double q2dotmax = 0.3;
const double q3dotmax = 0.3;
const double q4dotmax = 0.4;
const double q5dotmax = 0.4;
const double q6dotmax = 0.4;
const double dt_traj = 0.02; // time step for trajectory interpolation
const double SPEED_SCALE_FACTOR= 0.5; // go this fraction of speed from above maxes
std::string g_arm7dof_jnt_names[]={"joint0","joint1","joint2","joint3","joint4","joint5","joint6"};
const int arm7dof_NJNTS=7;

class Arm7dof_traj_streamer
{
public:
    Arm7dof_traj_streamer(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    void cmd_pose_right(Vectorq7x1 qvec );
    //void stuff_trajectory( std::vector<Vectorq7x1> qvecs, trajectory_msgs::JointTrajectory &new_trajectory);
    void stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory); 

 
    void pub_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory);
    Eigen::VectorXd get_q_vec_Xd() {return q_vec_Xd_;};
    Vectorq7x1 get_qvec() {return q_vec_;};  
    void pub_arm_trajectory_init();
    sensor_msgs::JointState get_joint_states() { return joint_states_;}
 private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber joint_state_sub_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer traj_interp_stat_client_;
    //ros::ServiceClient traj_interp_stat_client_;
    //ros::Publisher  joint_cmd_pub_;
    ros::Publisher  traj_pub_;
    vector<int> joint_indices_;   


    Vectorq7x1 q_vec_; //
    Eigen::VectorXd q_vec_Xd_;
    Vectorq7x1 qdot_max_vec_; // velocity constraint on each joint for interpolation
    sensor_msgs::JointState joint_states_; // copy from robot/joint_states subscription 
    //std_srvs::Trigger traj_status_srv_;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    void map_arm_joint_indices(vector<string> joint_names);
    void jointStatesCb(const sensor_msgs::JointState& js_msg); //prototype for callback of joint-state messages
    //void map_arms_joint_indices(vector<string> joint_names);
    double transition_time(Vectorq7x1 dqvec);
    double transition_time(Eigen::VectorXd dqvec);
    //prototype for callback for example service
    //bool serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif
