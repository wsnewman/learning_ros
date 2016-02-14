// example_steering_algorithm.h header file //
// wsn; Feb, 2015
// include this file in "example_steering_algorithm.cpp"

#ifndef MOBOT_NL_STEERING_H_
#define MOBOT_NL_STEERING_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

const double UPDATE_RATE = 100.0; // choose the update rate for steering controller
const double K_PSI= 5.0; // control gains for steering
const double K_LAT_ERR_THRESH = 3.0;
// dynamic limitations:  
const double MAX_SPEED = 1.0; // m/sec; tune this
const double MAX_OMEGA = 1.0; // rad/sec; tune this


// define a class, including a constructor, member variables and member functions
class SteeringController
{
public:
    SteeringController(ros::NodeHandle* nodehandle); 
    void mobot_nl_steering(); // use state and desired state to compute twist command, and publish it
    double psi_strategy(double); //computes strategic heading from lateral path-following error
    double omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);   
    double min_dang(double dang);  
    double sat(double x);
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber and publisher
    ros::Subscriber des_state_subscriber_; //publisher for states corresponding to ideal path following
    ros::Subscriber current_state_subscriber_; //topic to receive estimates of current robot state
    //initially, get state values from gazebo; need to replace with a sensor-based localization node
    
    ros::Publisher cmd_publisher_; // sends twist commands to cmd_vel topic
    ros::Publisher lat_err_publisher_; //publish values for debug visualization
    ros::Publisher heading_publisher_;
    ros::Publisher heading_cmd_publisher_;
    
    geometry_msgs::Twist twist_cmd_; 
    
    // variables used in the NL steering algorithm:
    double psi_cmd_; //computed heading command
    double lateral_err_; //path lateral offset error
    
    double current_speed_des_;
    double current_omega_des_;
    
    //state variables, (x,y,psi) and (speed, omega)
    double state_x_;
    double state_y_;
    double state_psi_;
    double state_speed_;
    double state_omega_;
    
    geometry_msgs::Quaternion state_quat_; 
    
    //state values from desired state; these will get filled in by desStateCallback 
    double des_state_x_;
    double des_state_y_;
    double des_state_psi_;   
        
    double des_state_speed_;
    double des_state_omega_;

    geometry_msgs::Quaternion des_state_quat_; 
    geometry_msgs::Pose des_state_pose_; 
        
    // private member methods:
    void initializeSubscribers(); 
    void initializePublishers();
 
    void gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose);
    void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);    
}; // end of class definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
