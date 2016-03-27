#include<traj_builder/traj_builder.h>

//This library contains functions to build simple navigation trajectories.
//The main function is: build_point_and_go_traj().  This function takes
// arguments of a start pose, a goal pose, and a reference to a vector of
// nav_msgs::Odom objects, which it will fill with a sequence of states that
// correspond to a smooth and executable trajectory from start to goal.
//This simple function will build a trajectory that does the following:
//  *find heading from start to goal coordinates
//  *compute a spin-in-place trajectory to point the robot towards the goal pose
//  *compute a forward-motion trajectory to move straight towards the goal pose
//The trajectories are stored in a vector of Odometry objects, sampled every time
// step dt_ (defaults to 20ms, but settable via fnc set_dt(double dt) ).
//The Odom objects contain incremental states of x, y, orientation, translational
//  velocity and rotational velocity
//For the spin and translation trajectories, these are either triangular-velocity
// profiles, or trapezoidal-velocity profiles, determined by speed and accel params.
//These parameters are set to default values by the constructor, but they can be 
// overridden via corresponding "set" functions.

//Note that for the spin-in-place behavior, the goal orientation is the direction
// from start coords to goal coords.  Thus, the orientation in the goal pose is
// ignored.  HOWEVER, if the start and goal poses have nearly identical (x,y) values,
// (as determined by the parameter "path_move_tol_"), then the goal pose is 
// interpreted to contain the desired goal heading.  Thus, if one wants to
// travel to a pose and reach both the specified coordinates and orientation, 
// simply repeat the last pose value as an additional subgoal, and this will
// lead to a computed trajectory that concludes with reorienting to the desired
// final heading.

// see example program: traj_builder_example_main.cpp for illustration of how to 
// use this library.


//constructor: fill in default param values (changeable via "set" fncs)
TrajBuilder::TrajBuilder()  {
    dt_ = default_dt; //0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
    //dynamic parameters: should be tuned for target system
    accel_max_ = default_accel_max; //0.5; //1m/sec^2
    alpha_max_ = default_alpha_max; //0.2; //1 rad/sec^2
    speed_max_ = default_speed_max; //1.0; //1 m/sec
    omega_max_ = default_omega_max; //1.0; //1 rad/sec
    path_move_tol_ = default_path_move_tol; //0.01; // if path points are within 1cm, fuggidaboutit   

    //define a halt state; zero speed and spin, and fill with viable coords
    halt_twist_.linear.x = 0.0;
    halt_twist_.linear.y = 0.0;
    halt_twist_.linear.z = 0.0;
    halt_twist_.angular.x = 0.0;
    halt_twist_.angular.y = 0.0;
    halt_twist_.angular.z = 0.0;
}

//fnc to choose shortest angular distance for motion dang, considering periodicity
double TrajBuilder::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}

// saturation function; limits values to range -1 to 1
double TrajBuilder::sat(double x) {
    if (x > 1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

//signum function: returns the sign of a value
double TrajBuilder::sgn(double x) {
    if (x > 0.0) {
        return 1.0;
    }
    if (x< 0.0) {
        return -1.0;
    }
    return 0.0;
}

//for planar motion, converts a quaternion to a scalar heading,
// where heading is measured CCW from x-axis
double TrajBuilder::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return psi;
}

//given a heading for motion on a plane (as above), convert this to a quaternion
geometry_msgs::Quaternion TrajBuilder::convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

//utility to fill a PoseStamped object from planar x,y,psi info
geometry_msgs::PoseStamped TrajBuilder::xyPsi2PoseStamped(double x, double y, double psi) {
    geometry_msgs::PoseStamped poseStamped; // a pose object to populate
    poseStamped.pose.orientation = convertPlanarPsi2Quaternion(psi); // convert from heading to corresponding quaternion
    poseStamped.pose.position.x = x; // keep the robot on the ground!
    poseStamped.pose.position.y = y; // keep the robot on the ground!    
    poseStamped.pose.position.z = 0.0; // keep the robot on the ground!  
    return poseStamped;
}

//here are the main traj-builder fncs:
//for spin-in-place motion that would hit maximum angular velocity, construct 
// trapezoidal angular velocity profile
void TrajBuilder::build_trapezoidal_spin_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    double t_ramp = omega_max_/ alpha_max_;    
    double ramp_up_dist = 0.5 * alpha_max_ * t_ramp*t_ramp;
    double cruise_distance = fabs(dpsi) - 2.0 * ramp_up_dist; //delta-angle to spin at omega_max        
    int npts_ramp = round(t_ramp / dt_);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    
    //ramp up omega (positive or negative);
    double t = 0.0;
    double accel = sgn(dpsi) * alpha_max_;
    double omega_des = 0.0;
    double psi_des = psi_start;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        omega_des = accel*t;
        des_state.twist.twist.angular.z = omega_des; //update rotation rate
        //update orientation
        psi_des = psi_start + 0.5 * accel * t*t;
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //now cruise for distance cruise_distance at const omega
    omega_des = sgn(dpsi)*omega_max_;
    des_state.twist.twist.angular.z  = sgn(dpsi)*omega_max_;
    double t_cruise = cruise_distance / omega_max_;
    int npts_cruise = round(t_cruise / dt_);
    for (int i = 0; i < npts_cruise; i++) {
        //Euler one-step integration
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }    
    //ramp down omega to halt:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel*dt_; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where desired, and at rest:
    des_state.pose.pose = end_pose.pose; //
    des_state.twist.twist = halt_twist_; // insist on full stop
    vec_of_states.push_back(des_state);
}

//upper-level function to construct a spin-in-place trajectory
// this function decides if triangular or trapezoidal angular-velocity
// profile is needed
void TrajBuilder::build_spin_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    //decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("rotational spin distance = %f", dpsi);
    double ramp_up_time = omega_max_/ alpha_max_;    
    double ramp_up_dist = 0.5 * alpha_max_ * ramp_up_time*ramp_up_time;
    //decide on triangular vs trapezoidal:
    if (fabs(dpsi) < 2.0 * ramp_up_dist) { //delta-angle is too short for trapezoid
        build_triangular_spin_traj(start_pose, end_pose, vec_of_states);
    } else {
        build_trapezoidal_spin_traj(start_pose, end_pose, vec_of_states);
    }   
}

//fnc to build a pure straight-line motion trajectory
//determines if a triangular or trapezoidal velocity profile is needed
void TrajBuilder::build_travel_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    //decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double trip_len = sqrt(dx * dx + dy * dy);
    double ramp_up_dist = 0.5 * speed_max_ * speed_max_ / alpha_max_;
    ROS_INFO("trip len = %f", trip_len);
    if (trip_len < 2.0 * ramp_up_dist) { //length is too short for trapezoid
        build_triangular_travel_traj(start_pose, end_pose, vec_of_states);
    } else {
        build_trapezoidal_travel_traj(start_pose, end_pose, vec_of_states);
    }
}

//given that straight-line, trapezoidal velocity profile trajectory is needed,
// construct the sequence of states (positions and velocities) to achieve the
// desired motion, subject to speed and acceleration constraints
void TrajBuilder::build_trapezoidal_travel_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = speed_max_ / accel_max_;
    double ramp_up_dist = 0.5 * accel_max_ * t_ramp*t_ramp;
    double cruise_distance = trip_len - 2.0 * ramp_up_dist; //distance to travel at v_max 
    ROS_INFO("t_ramp =%f",t_ramp);
    ROS_INFO("ramp-up dist = %f",ramp_up_dist);
    ROS_INFO("cruise distance = %f",cruise_distance);
    //start ramping up:
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    int npts_ramp = round(t_ramp / dt_);
    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //now cruise for distance cruise_distance at const speed
    speed_des = speed_max_;
    des_state.twist.twist.linear.x = speed_des;
    double t_cruise = cruise_distance / speed_max_;
    int npts_cruise = round(t_cruise / dt_);
    ROS_INFO("t_cruise = %f; npts_cruise = %d",t_cruise,npts_cruise);
    for (int i = 0; i < npts_cruise; i++) {
        //Euler one-step integration
        x_des += speed_des * dt_ * cos(psi_des);
        y_des += speed_des * dt_ * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= accel_max_*dt_; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration        
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

// constructs straight-line trajectory with triangular velocity profile,
// respective limits of velocity and accel
void TrajBuilder::build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / accel_max_);
    int npts_ramp = round(t_ramp / dt_);
    double v_peak = accel_max_*t_ramp; // could consider special cases for reverse motion
    double d_vel = alpha_max_*dt_; // incremental velocity changes for ramp-up

    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= accel_max_*dt_; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration        
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

void TrajBuilder::build_triangular_spin_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("spin traj: psi_start = %f; psi_end = %f; dpsi= %f", psi_start, psi_end, dpsi);
    double t_ramp = sqrt(fabs(dpsi) / alpha_max_);
    int npts_ramp = round(t_ramp / dt_);
    double psi_des = psi_start; //start from here
    double omega_des = 0.0; // assumes spin starts from rest; 
    // position of des_state will not change; only orientation and twist
    double t = 0.0;
    double accel = sgn(dpsi) * alpha_max_; //watch out for sign: CW vs CCW rotation
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        omega_des = accel*t;
        des_state.twist.twist.angular.z = omega_des; //update rotation rate
        //update orientation
        psi_des = psi_start + 0.5 * accel * t*t;
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel*dt_; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

//this function would be useful for planning a need for sudden braking
//compute trajectory corresponding to applying max prudent decel to halt
void TrajBuilder::build_braking_traj(geometry_msgs::PoseStamped start_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    //FINISH ME!

}

//main fnc of this library: constructs a spin-in-place reorientation to
// point to goal coords, then a straight-line trajectory from start to goal
//NOTE:  this function will clear out the vec_of_states vector of trajectory states.
// It does NOT append new planned states.  User beware.
void TrajBuilder::build_point_and_go_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    ROS_INFO("building point-and-go trajectory");
    nav_msgs::Odometry bridge_state;
    geometry_msgs::PoseStamped bridge_pose; //bridge end of prev traj to start of new traj
    vec_of_states.clear(); //get ready to build a new trajectory of desired states
    ROS_INFO("building rotational trajectory");
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double des_psi = atan2(dy, dx); //heading to point towards goal pose
    ROS_INFO("desired heading to subgoal = %f", des_psi);
    //bridge pose: state of robot with start_x, start_y, but pointing at next subgoal
    //  achieve this pose with a spin move before proceeding to subgoal with translational
    //  motion
    bridge_pose = start_pose;
    bridge_pose.pose.orientation = convertPlanarPsi2Quaternion(des_psi);
    ROS_INFO("building reorientation trajectory");
    build_spin_traj(start_pose, bridge_pose, vec_of_states); //build trajectory to reorient
    //start next segment where previous segment left off
    ROS_INFO("building translational trajectory");
    build_travel_traj(bridge_pose, end_pose, vec_of_states);
}
