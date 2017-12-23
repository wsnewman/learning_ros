// arm_motion_interface.h: 
// wsn,  Dec, 2017

#include <trajectory_msgs/JointTrajectory.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include <xform_utils/xform_utils.h>
#include <tf/transform_listener.h>
#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_motion_interface/cart_moveAction.h>
using namespace std;

//some needed robot-specific info:
struct ArmMotionInterfaceInits { 
    string urdf_base_frame_name; 
    string urdf_flange_frame_name; 
    string joint_states_topic_name;
    string traj_pub_topic_name; 
    vector<string> jnt_names;
    IKSolver * pIKSolver_arg;
    FwdSolver * pFwdSolver_arg;
    vector<double> q_lower_limits;
    vector<double> q_upper_limits;
    vector<double> qdot_max_vec;
    vector<double> q_home_pose;
};


class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    IKSolver * pIKSolver_;
    FwdSolver * pFwdSolver_;
    XformUtils xformUtils;
    
    double min_dt_traj_; 
    //create an action server, which will be called "cartMoveActionServer"
    //this service will accept goals in Cartesian coordinates
    actionlib::SimpleActionServer<arm_motion_interface::cart_moveAction> cart_move_as_;
    int NJNTS_; // this will get discovered by size of input arg for joint name vector
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    Eigen::VectorXd q_lower_limits_,q_upper_limits_,qdot_max_vec_,q_home_pose_; 

    ros::Publisher traj_publisher_; //<trajectory_msgs::JointTrajectory>;// = nh.advertise<trajectory_msgs::JointTrajectory>;//("joint_path_command", 1);   
    ros::Subscriber joint_states_subscriber_; 

    //messages to receive cartesian goals / return results:
    arm_motion_interface::cart_moveGoal cart_goal_;
    arm_motion_interface::cart_moveResult cart_result_;
    
    vector<string> jnt_names_;
    
    string    urdf_base_frame_name_; 
    string urdf_flange_frame_name_; 
    string joint_states_topic_name_;
    string traj_pub_topic_name_; 
    

    //callback fnc for joint-space action server to return result to this node:
    //void js_doneCb_(const actionlib::SimpleClientGoalState& state,
    //        const baxter_trajectory_streamer::trajResultConstPtr& result);
    //void armDoneCb_(const actionlib::SimpleClientGoalState& state,
    //    const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    //callback function to receive and act on cartesian move goal requests
    //this is the key method in this node;
    // can/should be extended to cover more motion-planning cases
    void executeCB(const actionlib::SimpleActionServer<arm_motion_interface::cart_moveAction>::GoalConstPtr& goal);

    double computed_arrival_time_; //when a move time is computed, result is stored here

    //poses, from goal message:
    geometry_msgs::PoseStamped goal_gripper_pose_; //cmd for  tool pose
    tf::StampedTransform generic_toolflange_frame_wrt_gripper_frame_stf_;
    tf::StampedTransform generic_gripper_frame_wrt_tool_flange_stf_;
    tf::StampedTransform base_link_wrt_system_ref_frame_stf_; 
    tf::StampedTransform  base_link_wrt_world_stf_;
    tf::StampedTransform  generic_gripper_frame_wrt_world_stf_;
    tf::StampedTransform  generic_gripper_frame_wrt_base_stf_; //current_gripper_frame_wrt_base_stf_
    //current tool poses w/rt base_frame:
    geometry_msgs::Pose current_gripper_pose_, current_flange_pose_; //cmd for tool pose
    geometry_msgs::PoseStamped current_gripper_stamped_pose_,current_flange_stamped_pose_; //cmd for rtool pose
    geometry_msgs::PoseStamped current_gripper_stamped_pose_wrt_world_;
    Eigen::Affine3d goal_gripper_affine_;
    Eigen::Affine3d goal_flange_affine_;

    //have not yet implemented gripper motion commands, as anticipated in goal message
    double gripper_open_close_cmd_; //gripper open/close commands
    bool vacuum_gripper_on_;
    unsigned short int command_mode_; //   

    vector<int> arm_joint_indices_;
    Eigen::VectorXd q_vec_; //alt representation of above, for convenience
    Eigen::VectorXd q_start_Xd_;

    Eigen::Affine3d affine_tool_wrt_base_,affine_flange_wrt_base_;

    Eigen::Affine3d A_tool_wrt_flange_;

    double arrival_time_;
    bool path_is_valid_;
    bool busy_working_on_a_request_;

    // vec to contain optimal path from planners
    std::vector<Eigen::VectorXd> optimal_path_; 
    trajectory_msgs::JointTrajectory des_trajectory_; //  trajectory object

    Eigen::VectorXd last_arm_jnt_cmd_;

    //some handy constants...
    Eigen::Matrix3d R_gripper_down_;
    //Eigen::VectorXd q_pre_pose_;
    Eigen::VectorXd q_pre_pose_Xd_;
    Eigen::VectorXd q_goal_pose_Xd_;

    //Irb120_fwd_solver fwd_solver_; //instantiate a forward-kinematics solver 
    CartTrajPlanner *pCartTrajPlanner_; // from cartesian trajectory planner library

    // key method: invokes motion from pre-planned trajectory
    // this is a private method, to try to protect it from accident or abuse
    void execute_planned_move(void);
    void jointStatesCb(const sensor_msgs::JointState& joint_states); //prototype for callback of joint_states subscriber
    void map_arm_joint_indices(vector<string> joint_names);
    void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory);
    double min_transition_time(Eigen::VectorXd dqvec);
    bool plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start, Eigen::VectorXd q_goal);
    void compute_tool_stamped_pose(void);


    Eigen::Affine3d a_tool_start_, a_tool_end_;
    //ros::Publisher  minimal_publisher_;
    //ros::Publisher  display_traj_pub_; 
    Eigen::VectorXd q_vec_arm_Xd_;
    
    Eigen::Vector3d delta_p_;
    Eigen::VectorXd q_vec_start_rqst_;
    Eigen::VectorXd q_vec_end_rqst_;
    Eigen::VectorXd q_vec_start_resp_;
    Eigen::VectorXd q_vec_end_resp_;
    Eigen::Affine3d a_flange_end_;

    double time_scale_stretch_factor_;
    bool finished_before_timeout_;

public:
    
    //ArmMotionInterface(ros::NodeHandle*, string, string, string, string, vector<string>, IKSolver *,FwdSolver *); //define the body of the constructor outside of class definition
    //ArmMotionInterface armMotionInterface(&nh,urdf_base_frame_name,urdf_flange_frame_name, joint_states_topic_name,traj_pub_topic_name,jnt_names);    
    ArmMotionInterface(ros::NodeHandle*, ArmMotionInterfaceInits); //define the body of the constructor outside of class definition

    ~ArmMotionInterface(void) {
    }
    tf::TransformListener* tfListener_; //make listener available to main;
    
    //handy utilities, primarily used internally, but publicly accessible
    //Eigen::Affine3d transformTFToEigen(const tf::Transform &t);
    //Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
/*  
    void display_affine(Eigen::Affine3d affine);



    //Eigen::VectorXd get_jspace_start_(void); //choose between most recent cmd, or current jnt angs


    //the following methods correspond to command codes, via action message goals
    //Eigen::VectorXd get_joint_angles(void);
    //get joint angles, compute fwd kin, convert result to a stamped pose
    // put answer in current_gripper_stamped_pose_
    void compute_tool_stamped_pose(void); //helper for GET_TOOL_POSE
    void compute_flange_stamped_pose(void); //helper for GET_FLANGE_POSE
    //void compute_left_tool_stamped_pose(void); 
    void compute_tool_stamped_pose_wrt_world(void);

    bool plan_path_current_to_goal_gripper_pose(); //uses goal.des_pose_gripper to plan a cartesian path
    //bool plan_path_current_to_goal_flange_pose(); //interprets goal.des_pose_flange  as a des FLANGE pose to plan a cartesian path
    //plan a joint-space path from current jspace pose to some soln of desired toolflange cartesian pose
    bool plan_jspace_path_current_to_cart_gripper_pose();
    //bool plan_fine_path_current_to_goal_flange_pose(); //interprets goal.des_pose_flange as a des FLANGE pose to plan a cartesian path
    bool plan_fine_path_current_to_goal_gripper_pose();
    //for 
    bool plan_path_current_to_goal_dp_xyz(); //plans cartesian motion by specified 3-D displacement at fixed orientation
    bool plan_cartesian_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p); //helper for above

    //following used inPLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE
    // and PLAN_JSPACE_PATH_CURRENT_TO_QGOAL
    //bool plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start, Eigen::VectorXd q_goal);
    bool plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start_Xd, Eigen::VectorXd q_goal_Xd);
    //bool jspace_path_planner_current_to_affine_goal(Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    void rescale_planned_trajectory_time(double time_stretch_factor);
    void set_arrival_time_planned_trajectory(double arrival_time);
    bool refine_cartesian_path_soln();
    Eigen::Affine3d xform_gripper_pose_to_affine_flange_wrt_base(geometry_msgs::PoseStamped des_pose_gripper);

    geometry_msgs::PoseStamped get_current_gripper_stamped_pose_wrt_world() { return current_gripper_stamped_pose_wrt_world_; };
*/  
};
