#ifndef TRAJ_BUILDER_H_
#define TRAJ_BUILDER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>

class TrajBuilder
{
private:
    ros::NodeHandle nh_;
//constants and parameters: changeable via public "set" fncs
    double dt_;
    double accel_max_;
    double alpha_max_;
    double speed_max_;
    double omega_max_;
    double path_move_tol_;
    
    //member vars of class
    geometry_msgs::Twist halt_twist_;      

public:
    TrajBuilder(ros::NodeHandle* nodehandle); //constructor
    void set_dt(double dt) { dt_ = dt; }
    void set_accel_max(double accel) {accel_max_= accel;}
    void set_alpha_max(double alpha) {alpha_max_= alpha;}
    void set_speed_max(double speed) {speed_max_= speed;}
    void set_omega_max(double omega) {omega_max_ = omega;}
    void set_path_move_tol_(double tol) {path_move_tol_ = tol;}
    
    //useful utilties:
    double min_dang(double dang);
    double sat(double x);
    double sgn(double x);
    double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi);
    
    //utility to fill a PoseStamped object from planar x,y,phi info
    geometry_msgs::PoseStamped xyPsi2PoseStamped(double x, double y, double psi);
    
    //here are the main traj-builder fncs:
    void build_trapezoidal_spin_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_triangular_spin_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_spin_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_travel_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_trapezoidal_travel_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);  
   void build_point_and_go_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);
   void build_braking_traj(geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                std::vector<nav_msgs::Odometry> &vec_of_states);    
  
}; 

#endif 
