//generic_arm_motion_interface.h:

// uses library of arm-motion planning functions
//#include <cartesian_planner/irb120_cartesian_planner.h>
#include <generic_cartesian_planner/cart_moveAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
//#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <ur_fk_ik/ur_kin.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/JointState.h>
//#include<moveit_msgs/DisplayTrajectory.h>
#include<generic_cartesian_planner/generic_cartesian_planner.h>

#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

using namespace std;

/*
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;


//vector<string> g_jnt_names;
//const double SPEED_SCALE_FACTOR=1.0; //increase this to slow down motions

//const double ARM_ERR_TOL = 0.1; // tolerance btwn last joint commands and current arm pose
// used to decide if last command is good start point for new path

const double dt_traj = 0.02; // time step for trajectory interpolation

bool g_js_doneCb_flag = true;
bool g_spray_on = false;

double transition_time(Eigen::VectorXd dqvec) {
    double t_max = SPEED_SCALE_FACTOR*fabs(dqvec[0]) / g_qdot_max_vec[0];
    //ROS_INFO("t0 = %f; dqvec[0] = %f; g_qdot_max_vec[0] = %f",t_max,dqvec[0],g_qdot_max_vec[0]);
    //cout<<"qdot max: "<<g_qdot_max_vec.transpose()<<endl; //xxx
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = SPEED_SCALE_FACTOR*fabs(dqvec[i]) / g_qdot_max_vec[i];
        //ROS_INFO("ti = %f; dqvec[i] = %f; g_qdot_max_vec[i] = %f",ti,dqvec[i],g_qdot_max_vec[i]);
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_jnt_names
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_jnt_names[i].c_str());
    }

    //try imposing a time delay on first point to work around ros_controller complaints
    double t_start=0.05;

    new_trajectory.header.stamp = ros::Time::now(); //+ros::Duration(t_start);  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = t_start;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];
    trajectory_point1.positions.clear();
    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<VECTOR_DIM;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }
}

//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
}

void sprayOnOffCb(const std_msgs::Bool   &onoff_msg) {
   g_spray_on = onoff_msg.data;
   if (g_spray_on) ROS_INFO("rcvd cmd spray state ON");
   else ROS_INFO("rcvd cmd spray state OFF");
}
*/
class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    XformUtils xformUtils;
    //create an action server, which will be called "cartMoveActionServer"
    //this service will accept goals in Cartesian coordinates
    actionlib::SimpleActionServer<cartesian_planner::cart_moveAction> cart_move_as_;
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    //void set_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //Eigen::VectorXd g_q_vec_arm_Xd; //.resize(VECTOR_DIM); //made this one global   
    // create an action client, which will send joint-space goals to the trajectory interpolator service
 
    
    //actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> traj_streamer_action_client_;
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    //ros::Publisher traj_publisher_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
    ros::Publisher traj_publisher_; //<trajectory_msgs::JointTrajectory>;// = nh.advertise<trajectory_msgs::JointTrajectory>;//("joint_path_command", 1);    
    //messages to receive cartesian goals / return results:
    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_;

    //callback fnc for joint-space action server to return result to this node:
    //void js_doneCb_(const actionlib::SimpleClientGoalState& state,
    //        const baxter_trajectory_streamer::trajResultConstPtr& result);
    //void armDoneCb_(const actionlib::SimpleClientGoalState& state,
    //    const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    //callback function to receive and act on cartesian move goal requests
    //this is the key method in this node;
    // can/should be extended to cover more motion-planning cases
    void executeCB(const actionlib::SimpleActionServer<cartesian_planner::cart_moveAction>::GoalConstPtr& goal);
    // key method: invokes motion from pre-planned trajectory
    // this is a private method, to try to protect it from accident or abuse
    void execute_planned_move(void);
    
    
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

    Eigen::VectorXd q_vec_; //alt representation of above, for convenience
    Eigen::VectorXd q_start_Xd_;

    Eigen::Affine3d affine_tool_wrt_base_,affine_flange_wrt_base_;

    Eigen::Affine3d A_tool_wrt_flange_;

    double arrival_time_;
    bool path_is_valid_;

    // vec to contain optimal path from planners
    std::vector<Eigen::VectorXd> optimal_path_; 
    trajectory_msgs::JointTrajectory des_trajectory_; //  trajectory object

    Eigen::VectorXd last_arm_jnt_cmd_;

    //some handy constants...
    Eigen::Matrix3d R_gripper_down_;
    //Eigen::VectorXd q_pre_pose_;
    Eigen::VectorXd q_pre_pose_Xd_;
    Eigen::VectorXd q_goal_pose_Xd_;

    //sensor_msgs::JointState joint_states_;
    //moveit_msgs::DisplayTrajectory display_trajectory_;

    //Irb120_fwd_solver fwd_solver_; //instantiate a forward-kinematics solver 
    CartTrajPlanner cartTrajPlanner_; // from cartesian trajectory planner library



    //the rest of these private methods and variables are obsolete, service related
    // member methods as well:
    //void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    //void initializePublishers();
    //internal flags describing state of this server; these might go away
    // some objects to support subscriber, service, and publisher
    ros::ServiceServer arm_motion_interface_service_;
    //cartesian_planner::armNavSrvMsgRequest request_; 
    int path_id_;
    Eigen::Affine3d a_tool_start_, a_tool_end_;
    //ros::Publisher  minimal_publisher_;
    //ros::Publisher  display_traj_pub_; 
    Eigen::Vector3d delta_p_;
    Eigen::VectorXd q_vec_start_rqst_;
    Eigen::VectorXd q_vec_end_rqst_;
    Eigen::VectorXd q_vec_start_resp_;
    Eigen::VectorXd q_vec_end_resp_;
    Eigen::Affine3d a_flange_end_;

    double time_scale_stretch_factor_;
    bool received_new_request_; // = false;
    bool busy_working_on_a_request_; // = false;
    bool finished_before_timeout_;
    //void initializeServices();     
    //bool cartMoveSvcCB(cartesian_planner::armNavSrvMsgRequest& request, cartesian_planner::armNavSrvMsgResponse& response);
    //void pack_qstart(cartesian_planner::armNavSrvMsgResponse& response);
    //void pack_qend(cartesian_planner::armNavSrvMsgResponse& response);
     //   baxter_trajectory_streamer::trajGoal js_goal_; //goal message to send to joint-space interpolator server
    //baxter_trajectory_streamer::trajResult js_result_; // server will populate this result, when done w/ goal
    control_msgs::FollowJointTrajectoryGoal js_goal_; //consistent goal message for UR action service

public:
    ArmMotionInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionInterface(void) {
    }
    tf::TransformListener* tfListener_; //make listener available to main;
    
    //handy utilities, primarily used internally, but publicly accessible
    //Eigen::Affine3d transformTFToEigen(const tf::Transform &t);
    //Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
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

};

