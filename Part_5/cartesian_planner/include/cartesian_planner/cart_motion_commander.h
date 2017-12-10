#ifndef CART_MOTION_COMMANDER_H
#define	CART_MOTION_COMMANDER_H
#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <xform_utils/xform_utils.h>
//using namespace std;


//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmMotionCommander {
private:
    //ros::NodeHandle *nh_;
    //XformUtils xformUtils_;
    //messages to send/receive cartesian goals / results:
    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_;    
    std::vector <double> q_vec_; //holder for arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<cartesian_planner::cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,
    const cartesian_planner::cart_moveResultConstPtr& result);
    bool got_done_callback_;
public:
        ArmMotionCommander(); //define the body of the constructor outside of class definition

    ~ArmMotionCommander(void) {
    }
    void send_test_goal(void);
    int plan_move_to_waiting_pose(void); 
    int plan_path_current_to_goal_gripper_pose(geometry_msgs::PoseStamped des_pose);
    int plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);
    int plan_jspace_path_current_to_cart_gripper_pose(geometry_msgs::PoseStamped des_pose);    
    int execute_planned_path(void);
    
    int request_q_data(void);
    int request_tool_pose(void);
    geometry_msgs::PoseStamped get_tool_pose_stamped(void) { return tool_pose_stamped_;};
    Eigen::VectorXd get_joint_angles(void); 
    int plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec);  
    bool cb_received_in_time(double max_wait_time);
    void time_rescale_planned_trajectory(double time_scale_factor);
    void set_arrival_time_planned_trajectory(double arrival_time);
};
#endif
