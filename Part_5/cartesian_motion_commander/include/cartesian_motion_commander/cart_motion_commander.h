#ifndef CART_MOTION_COMMANDER_H
#define	CART_MOTION_COMMANDER_H
#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <trajectory_msgs/JointTrajectory.h>


#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
//#include <xform_utils/xform_utils.h>
//using namespace std;

const double MAX_WAIT_TIME=10.0; //avoid deadlock--wait for max time for server response


//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class CartMotionCommander {
private:
    //ros::NodeHandle *nh_;
    //XformUtils xformUtils_;
    //messages to send/receive cartesian goals / results:
    //arm_motion_action::arm_interfaceGoal::ARM_TEST_MODE:
    
    arm_motion_action::arm_interfaceGoal cart_goal_;
    arm_motion_action::arm_interfaceResult cart_result_;    
    std::vector <double> q_vec_; //holder for arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<arm_motion_action::arm_interfaceAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,const arm_motion_action::arm_interfaceResultConstPtr& result);
    int send_planning_goal_get_result(double t_wait);
    int request_q_data(void);
    bool got_done_callback_;
    int NJNTS_;
    
public:
        CartMotionCommander(); //define the body of the constructor outside of class definition

    ~CartMotionCommander(void) {
    }
    void set_njnts(int njnts) { NJNTS_ = njnts; };
    
    bool cb_received_in_time(double max_wait_time);    
    void send_test_goal(void);
    Eigen::VectorXd get_joint_angles(void); 
    int clear_multi_traj_plan(void);
    
    
    geometry_msgs::PoseStamped get_tool_pose_stamped(void); // { return tool_pose_stamped_;};    
    int execute_planned_traj(void);  
    int execute_traj_nseg(int iseg);
    int execute_traj_nseg(int iseg,double desired_move_time);

    
    int plan_jspace_traj_current_to_waiting_pose(int nsteps, double arrival_time); //traj current pose to a jspace home pose
    int plan_jspace_traj_current_to_qgoal(int nsteps, double arrival_time,Eigen::VectorXd q_goal); //traj current to a specified jspace pose
    //int plan_jspace_traj_qstart_to_qend(int nsteps, double arrival_time,Eigen::VectorXd q_start,Eigen::VectorXd q_goal);   //jspace traj from specified q_start to q_end
    int plan_jspace_traj_current_to_tool_pose(int nsteps, double arrival_time,geometry_msgs::PoseStamped des_pose);   //computes a jspace traj from start pose to some IK soln of desired tool pose
    
    int plan_cartesian_traj_current_to_des_tool_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose);
    int plan_cartesian_traj_qstart_to_des_tool_pose(int nsteps, double arrival_time, Eigen::VectorXd q_start, geometry_msgs::PoseStamped des_pose);
    //the next command plans from previous trajectory end jspace pose
    int plan_cartesian_traj_qprev_to_des_tool_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose);

    int append_multi_traj_cart_segment(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose);
    //uint8 APPEND_MULTI_TRAJ_JSPACE_SEGMENT = 28 not implemented yet; not needed?
    //int append_multi_traj_jspace_segment()
    
    //bool plan_jspace_traj_qstart_to_des_tool_pose(Eigen::VectorXd  q_start,int nsteps,double arrival_time,geometry_msgs::PoseStamped des_pose);

/*
 uint8 PLAN_JSPACE_TRAJ_CURRENT_TO_WAITING_POSE=20
uint8 PLAN_JSPACE_TRAJ_CURRENT_TO_QGOAL = 21
uint8 PLAN_JSPACE_TRAJ_CURRENT_TO_CART_TOOL_POSE = 22 #plan a joint-space path from current arm pose to some IK soln of Cartesian goal

 */
    
    /*
 
    int plan_path_current_to_goal_gripper_pose(geometry_msgs::PoseStamped des_pose);
    int plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);
 
    //bool plan_jspace_traj_qstart_to_affine_flange_goal(Eigen::VectorXd q_start, Eigen::Affine3d goal_flange_affine, int nsteps, 
    //      double arrival_time,optimal_path);

    bool plan_jspace_path_qstart_to_cart_gripper_pose(Eigen::VectorXd q_start, int nsteps, geometry_msgs::PoseStamped des_pose);
    //bool plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);

    //bool CartTrajPlanner::plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory) {


    
    int request_q_data(void);
    int request_tool_pose(void);
    int plan_jspace_traj_current_to_qgoal(Eigen::VectorXd q_des_vec);  

    void time_rescale_planned_trajectory(double time_scale_factor);

     * */
};
#endif
