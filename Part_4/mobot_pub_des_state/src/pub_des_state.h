#ifndef PUB_DES_STATE_H_
#define PUB_DES_STATE_H_

#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mobot_pub_des_state/path.h>
#include <std_msgs/Float64.h>

//constants and parameters:
const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system
const double accel_max = 0.5; //1m/sec^2
const double alpha_max = 0.2; // rad/sec^2
const double speed_max = 1.0; //1 m/sec
const double omega_max = 1.0; //1 rad/sec
const double path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;

class DesStatePublisher {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    //some class member variables:
    nav_msgs::Path path_;
    std::vector<nav_msgs::Odometry> des_state_vec_;
    nav_msgs::Odometry des_state_;
    nav_msgs::Odometry halt_state_;
    nav_msgs::Odometry seg_end_state_;
    nav_msgs::Odometry seg_start_state_;
    nav_msgs::Odometry current_des_state_;
    geometry_msgs::Twist halt_twist_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped end_pose_;
    geometry_msgs::PoseStamped current_pose_;
    std_msgs::Float64 float_msg_;
    double des_psi_;
    std::queue<geometry_msgs::PoseStamped> path_queue_; //a C++ "queue" object, stores vertices as Pose points in a FIFO queue
    int motion_mode_;
    bool e_stop_trigger_; //these are intended to enable e-stop via a service
    bool e_stop_reset_;
    int traj_pt_i_;
    int npts_traj_;
    double dt_;
    //dynamic parameters: should be tuned for target system
    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_; 

    // some objects to support service and publisher
    ros::ServiceServer estop_service_;
    ros::ServiceServer estop_clear_service_;
    ros::ServiceServer flush_path_queue_;
    ros::ServiceServer append_path_;
    
    ros::Publisher desired_state_publisher_;
    ros::Publisher des_psi_publisher_;
    
    //a trajectory-builder object; 
    TrajBuilder trajBuilder_; 

    // member methods:
    void initializePublishers();
    void initializeServices();
    bool estopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool clearEstopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool flushPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool appendPathQueueCB(mobot_pub_des_state::pathRequest& request,mobot_pub_des_state::pathResponse& response);

public:
    DesStatePublisher(ros::NodeHandle& nh);//constructor
    int get_motion_mode() {return motion_mode_;}
    void set_motion_mode(int mode) {motion_mode_ = mode;}
    bool get_estop_trigger() { return e_stop_trigger_;}
    void reset_estop_trigger() { e_stop_trigger_ = false;}
    void set_init_pose(double x,double y, double psi);
    void pub_next_state();
    void append_path_queue(geometry_msgs::PoseStamped pose) { path_queue_.push(pose); }
    void append_path_queue(double x, double y, double psi) 
        { path_queue_.push(trajBuilder_.xyPsi2PoseStamped(x,y,psi)); }
};
#endif