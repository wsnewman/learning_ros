// ur10_cart_move_as: 
// wsn,  Nov, 2016
// action server to accept commands and perform planning and motion requests

//for UR10, reference tool0 for tool flange and base_link


//NOTE: ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE does: unpack_goal_pose(),
// which does:     a_flange_end_ = a_tool_end_*A_tool_wrt_flange_.inverse();
//  ik functions are w/rt desired flange frame

// uses library of arm-motion planning functions
#include <cartesian_planner/ur10_cartesian_planner.h>
//#include <cartesian_planner/ur10_cart_moveAction.h>
#include <cartesian_planner/cart_moveAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ur_fk_ik/ur_kin.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/UInt8.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/JointState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
const double SPEED_SCALE_FACTOR=1.0; //increase this to slow down motions

const double ARM_ERR_TOL = 0.1; // tolerance btwn last joint commands and current arm pose
// used to decide if last command is good start point for new path

const double dt_traj = 0.02; // time step for trajectory interpolation

bool g_js_doneCb_flag = true;
void set_ur_jnt_names() {
    g_ur_jnt_names.push_back("shoulder_pan_joint");
    g_ur_jnt_names.push_back("shoulder_lift_joint");
    g_ur_jnt_names.push_back("elbow_joint");
    g_ur_jnt_names.push_back("wrist_1_joint");
    g_ur_jnt_names.push_back("wrist_2_joint");
    g_ur_jnt_names.push_back("wrist_3_joint");
}
double transition_time(Eigen::VectorXd dqvec) {
    double t_max = SPEED_SCALE_FACTOR*fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = SPEED_SCALE_FACTOR*fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_ur_jnt_names
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
    }

    //try imposing a time delay on first point to work around ros_controller complaints
    double t_start=0.05;

    new_trajectory.header.stamp = ros::Time::now(); //+ros::Duration(t_start);  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = t_start;
    q_start = qvecs[0];
    q_end = qvecs[0];
    //cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

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
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
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
        j_name = g_ur_jnt_names[j]; //known name, in preferred order
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




class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    XformUtils xformUtils;
    //create an action server, which will be called "cartMoveActionServer"
    //this service will accept goals in Cartesian coordinates
    actionlib::SimpleActionServer<cartesian_planner::cart_moveAction> cart_move_as_;
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    //void set_ur_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //Eigen::VectorXd g_q_vec_arm_Xd; //.resize(VECTOR_DIM); //made this one global   
    // create an action client, which will send joint-space goals to the trajectory interpolator service
 
    
    //actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> traj_streamer_action_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    
    //messages to receive cartesian goals / return results:
    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_;

    //callback fnc for joint-space action server to return result to this node:
    //void js_doneCb_(const actionlib::SimpleClientGoalState& state,
    //        const baxter_trajectory_streamer::trajResultConstPtr& result);
    void armDoneCb_(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    //callback function to receive and act on cartesian move goal requests
    //this is the key method in this node;
    // can/should be extended to cover more motion-planning cases
    void executeCB(const actionlib::SimpleActionServer<cartesian_planner::cart_moveAction>::GoalConstPtr& goal);

    double computed_arrival_time_; //when a move time is computed, result is stored here

    //poses, from goal message:
    geometry_msgs::PoseStamped goal_gripper_pose_; //cmd for  tool pose
    tf::StampedTransform generic_toolflange_frame_wrt_gripper_frame_stf_;
    tf::StampedTransform generic_gripper_frame_wrt_tool_flange_stf_;
    tf::StampedTransform base_link_wrt_system_ref_frame_stf_; 
    //current tool poses w/rt base_frame:
    geometry_msgs::Pose current_gripper_pose_, current_flange_pose_; //cmd for tool pose
    geometry_msgs::PoseStamped current_gripper_stamped_pose_,current_flange_stamped_pose_; //cmd for rtool pose

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
    moveit_msgs::DisplayTrajectory display_trajectory_;

    //UR10IkSolver ur10_IK_solver_; // instantiate an IK solver
    UR10FwdSolver ur10_fwd_solver_; //instantiate a forward-kinematics solver 
    CartTrajPlanner cartTrajPlanner_; // from cartesian trajectory planner library

    // key method: invokes motion from pre-planned trajectory
    // this is a private method, to try to protect it from accident or abuse
    void execute_planned_move(void);

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
    bool refine_cartesian_path_soln();
    Eigen::Affine3d xform_gripper_pose_to_affine_flange_wrt_base(geometry_msgs::PoseStamped des_pose_gripper);

};
Eigen::Affine3d ArmMotionInterface::xform_gripper_pose_to_affine_flange_wrt_base(geometry_msgs::PoseStamped des_pose_gripper) {
    Eigen::Affine3d affine_flange_wrt_base;
    tf::StampedTransform flange_stf, flange_wrt_base_stf;
    geometry_msgs::PoseStamped flange_gmps, flange_wrt_base_gmps;
    //convert des gripper pose to a stamped transform, so we can do more transforms
    ROS_WARN("xform_gripper_pose_to_affine_flange_wrt_base: input pose-stamped: ");
    xformUtils.printStampedPose(des_pose_gripper);
    tf::StampedTransform gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(des_pose_gripper, "generic_gripper_frame"); 
    //convert to transform of corresponding tool flange w/rt whatever reference frame_id
    ROS_INFO("gripper_stf: ");
    xformUtils.printStampedTf(gripper_stf);
   
    bool mult_ok = xformUtils.multiply_stamped_tfs(gripper_stf,generic_toolflange_frame_wrt_gripper_frame_stf_,flange_stf);
    ROS_INFO("flange_stf");
    xformUtils.printStampedTf(flange_stf); 
    if (!mult_ok) { ROS_WARN("stf multiply not legal! "); } //should not happen
    //ROS_INFO("corresponding flange frame: ");
    //xformUtils.printStampedTf(flange_stf);
    //convert stamped ‘tf::StampedTransform to geometry_msgs::PoseStamped
    flange_gmps = xformUtils.get_pose_from_stamped_tf(flange_stf);
    //change the reference frame from whatever to "base":
    bool tferr=true;
    int ntries=0;   
    ros::Time now = ros::Time::now();    
    while (tferr) {
        tferr=false;
        try {
            tfListener_->waitForTransform("base_link", flange_gmps.header.frame_id,
                              now, ros::Duration(1.0)); //waits up to N sec for transform to be valid
                tfListener_->transformPose("base_link", flange_gmps, flange_wrt_base_gmps); 
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; transform problem; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    
    ROS_INFO("corresponding flange frame w/rt base frame: ");
    xformUtils.printStampedPose(flange_wrt_base_gmps);  
    //convert this to an affine.  parent and child frame id's are lost, so we'll have to remember what this means
    affine_flange_wrt_base = xformUtils.transformPoseToEigenAffine3d(flange_wrt_base_gmps);
    ROS_WARN("xform_gripper_pose_to_affine_flange_wrt_base returning affine: ");
    display_affine(affine_flange_wrt_base);
    xformUtils.printAffine(affine_flange_wrt_base);
    return affine_flange_wrt_base;
}    

void ArmMotionInterface::executeCB(const actionlib::SimpleActionServer<cartesian_planner::cart_moveAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB of ArmMotionInterface");
    cart_goal_ = *goal; // copy of goal held in member var
    command_mode_ = goal->command_code;
    ROS_INFO_STREAM("received command mode " << command_mode_);
    int njnts;

    switch (command_mode_) {
        //a simple "is-alive" test
        case cartesian_planner::cart_moveGoal::ARM_TEST_MODE:
            ROS_INFO("responding to request TEST_MODE: ");
            cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;
            
            //prepares a trajectory plan to move arm from current pose to pre-defined pose
        case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE:
            ROS_INFO("responding to request PLAN_PATH_CURRENT_TO_WAITING_POSE");
            q_start_Xd_ = g_q_vec_arm_Xd;//get_jspace_start_();
            //q_start=q_start_Xd; // convert to fixed-size vector;
            plan_jspace_path_qstart_to_qend(q_start_Xd_, q_pre_pose_Xd_);
            busy_working_on_a_request_ = false;
            break;   
         
        //computes a joint-space path from current pose to a gripper Cartesian destination
        case cartesian_planner::cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_CART_GRIPPER_POSE:  
            ROS_WARN("responding to request PLAN_JSPACE_PATH_CURRENT_TO_CART_GRIPPER_POSE");
            plan_jspace_path_current_to_cart_gripper_pose();   
            break;            
        
        //computes a Cartesian-space path from current pose to a desired gripper pose
        case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_GRIPPER_POSE:
            plan_path_current_to_goal_gripper_pose();
            break;

        //computes a Cartesian-space path from current pose with pure translation along
        //a specified 3-D displacement while keeping gripper orientation constant
        case cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ:
            plan_path_current_to_goal_dp_xyz();
            break;
        
        //for a computed trajectory, this fnc rescales the speed along the path
        case cartesian_planner::cart_moveGoal::TIME_RESCALE_PLANNED_TRAJECTORY:
            time_scale_stretch_factor_ = goal->time_scale_stretch_factor;
            rescale_planned_trajectory_time(time_scale_stretch_factor_);
            break;            

        //consults a pre-computed trajectory and invokes execution;
        case cartesian_planner::cart_moveGoal::EXECUTE_PLANNED_PATH: //assumes there is a valid planned path in optimal_path_
            ROS_INFO("responding to request EXECUTE_PLANNED_PATH");
            execute_planned_move();
            break;

         //the following functions are less generic--should avoid use of flange
         //reference frame or use of joint-space goals
            //looks up current arm joint angles and returns them to client
        case cartesian_planner::cart_moveGoal::GET_Q_DATA:
            ROS_INFO("responding to request GET_Q_DATA");
            //get_joint_angles(); 
            cart_result_.q_arm.resize(NJNTS);
            for (int i = 0; i < NJNTS; i++) {
                cart_result_.q_arm[i] = g_q_vec_arm_Xd[i];
            }
            cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;

        case cartesian_planner::cart_moveGoal::GET_TOOL_POSE:
            ROS_INFO("responding to request GET_TOOL_POSE");
            compute_tool_stamped_pose();
            cart_result_.current_pose_gripper = current_gripper_stamped_pose_;
            cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;

        case cartesian_planner::cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_QGOAL:
            ROS_INFO("responding to request PLAN_JSPACE_PATH_CURRENT_TO_QGOAL");
            q_start_Xd_ = g_q_vec_arm_Xd; //get_jspace_start_();
            q_goal_pose_Xd_.resize(NJNTS);
            njnts = goal->q_goal.size();
            if (njnts != NJNTS) {
                ROS_WARN("joint-space goal is wrong dimension");
                cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
            } else {
                for (int i = 0; i < NJNTS; i++) q_goal_pose_Xd_[i] = goal->q_goal[i];
                //q_start=q_start_Xd; // convert to fixed-size vector;
                plan_jspace_path_qstart_to_qend(q_start_Xd_, q_goal_pose_Xd_);
                busy_working_on_a_request_ = false;
            }
            break;

 
        default:
            ROS_WARN("this command mode is not defined: %d", command_mode_);
            cart_result_.return_code = cartesian_planner::cart_moveResult::COMMAND_CODE_NOT_RECOGNIZED;
            cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }
}


//CONSTRUCTOR: pass in a node handle and perform initializations (w/ initializers)

ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle),
cart_move_as_(*nodehandle, "cartMoveActionServer", boost::bind(&ArmMotionInterface::executeCB, this, _1), false),
action_client_("/arm_controller/follow_joint_trajectory", true)
 { // constructor
    ROS_INFO("in class constructor of ArmMotionInterface");
    g_q_vec_arm_Xd.resize(VECTOR_DIM);
    //initialize variables here, as needed
    q_pre_pose_Xd_.resize(VECTOR_DIM);
    q_pre_pose_Xd_ << 1.57, -1.57, -1.57, -1.57, 1.57, 0;
    //q_pre_pose_Xd_ = q_pre_pose_; // copy--in VectorXd format
    q_vec_start_rqst_.resize(NJNTS); // = q_pre_pose_; // 0,0,0,0,0,0,0; // make this a N-d vector
    q_vec_end_rqst_.resize(NJNTS); // = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_start_resp_.resize(NJNTS);// = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_end_resp_.resize(NJNTS);// = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    R_gripper_down_ = cartTrajPlanner_.get_R_gripper_down();

    //access constants defined in action message this way:
    command_mode_ = cartesian_planner::cart_moveGoal::ARM_TEST_MODE;

    received_new_request_ = false;
    busy_working_on_a_request_ = false;
    path_is_valid_ = false;
    path_id_ = 0;
    // can also do tests/waits to make sure all required services, topics, etc are alive

    //A_tool_wrt_flange_ = ur10_fwd_solver_.get_affine_tool_wrt_flange();
    tfListener_ = new tf::TransformListener;  //create a transform listener and assign its pointer
    bool tferr=true;
    int ntries=0;
    ROS_INFO("waiting for tf between generic gripper frame and tool flange...");

    while (tferr) {
        tferr=false;
        try {
                tfListener_->lookupTransform("generic_gripper_frame","tool0",  ros::Time(0), generic_toolflange_frame_wrt_gripper_frame_stf_);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();      
                ntries++;
                if (ntries>5) ROS_WARN("did you launch cartesian_planner/ur10_static_transforms.launch?");
            }   
    }
    ROS_INFO("tf is good for generic gripper frame w/rt tool flange");
    xformUtils.printStampedTf(generic_toolflange_frame_wrt_gripper_frame_stf_);
 
        tferr=true;
    ROS_INFO("waiting for tf between system_ref_frame and base_link...");

    while (tferr) {
        tferr=false;
        try {
                tfListener_->lookupTransform("system_ref_frame", "base_link", ros::Time(0), base_link_wrt_system_ref_frame_stf_);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good for system_ref_frame and base_link");    
    xformUtils.printStampedTf(base_link_wrt_system_ref_frame_stf_);
    
    //check that joint-space interpolator service is connected:
    // attempt to connect to the trajectory action server:
    //instantiate client of the arm server:
    //create an action client of UR's follow_joint_trajectory action server 
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
     //       arm_action_client("/arm_controller/follow_joint_trajectory", true);

    // attempt to connect to the server(s):
    ROS_INFO("waiting for arm-control server: ");
    bool server_exists = action_client_.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = action_client_.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  
    
    ROS_INFO("getting joint states: ");
    q_vec_ = g_q_vec_arm_Xd;//get_joint_angles(); //
    ROS_INFO("got joint states");
    last_arm_jnt_cmd_= q_vec_;

    ROS_INFO("starting action server: cartMoveActionServer ");
    cart_move_as_.start(); //start the server running
}

//callback fnc from joint-space trajectory streamer
//action server will respond to this callback when done
void ArmMotionInterface::armDoneCb_(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    g_js_doneCb_flag = true;
    
}


//handy utility, just to print data to screen for Affine objects

void ArmMotionInterface::display_affine(Eigen::Affine3d affine) {
    cout << "Affine origin: " << affine.translation().transpose() << endl;
    cout << "Affine rotation: " << endl;
    cout << affine.linear() << endl;
}

//handy fnc to get current  tool pose
// gets current joint angles, does fwd kin, includes tool xform
// converts result to a geometry_msgs::PoseStamped

void ArmMotionInterface::compute_tool_stamped_pose(void) {
    //get_joint_angles(); //will update q_vec
    q_vec_= g_q_vec_arm_Xd;
    affine_tool_wrt_base_ =
            ur10_fwd_solver_.fwd_kin_solve(q_vec_); //rtns pose w/rt base frame
    current_gripper_pose_ = xformUtils.transformEigenAffine3dToPose(affine_tool_wrt_base_);
    current_gripper_stamped_pose_.pose = current_gripper_pose_;
    current_gripper_stamped_pose_.header.stamp = ros::Time::now(); 
    current_gripper_stamped_pose_.header.frame_id = "base_link";
}

void ArmMotionInterface::compute_flange_stamped_pose(void) {
    //get_joint_angles(); //will update q_vec_Xd_ 
    q_vec_ = g_q_vec_arm_Xd;
    affine_flange_wrt_base_ =
            ur10_fwd_solver_.fwd_kin_solve(q_vec_); //rtns pose w/rt base frame
    current_flange_pose_ = xformUtils.transformEigenAffine3dToPose(affine_flange_wrt_base_);
    current_flange_stamped_pose_.pose = current_flange_pose_;
    current_flange_stamped_pose_.header.stamp = ros::Time::now();
    current_flange_stamped_pose_.header.frame_id = "base_link";
}

void ArmMotionInterface::execute_planned_move(void) {
    if (!path_is_valid_) {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        ROS_WARN("attempted to execute invalid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }

    // convert path to a trajectory:
    //stuff_trajectory(optimal_path_, des_trajectory_);
    des_trajectory_.header.stamp = ros::Time::now();
    js_goal_.trajectory = des_trajectory_;
    //computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    ROS_INFO("sending action request");
    ROS_INFO("computed arrival time is %f", computed_arrival_time_);
    busy_working_on_a_request_ = true;
    g_js_doneCb_flag = false;
    //arm_action_client.sendGoal(goal, &armDoneCb);
    action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::armDoneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    ROS_INFO("waiting on trajectory streamer...");
    while (!g_js_doneCb_flag) {
        ROS_INFO("...");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        cout<<"jnt angs: "<<g_q_vec_arm_Xd.transpose()<<endl;
    }
    //finished_before_timeout_ = traj_streamer_action_client_.waitForResult(ros::Duration(computed_arrival_time_ + 2.0));
    /*
    if (!finished_before_timeout_) {
        ROS_WARN("EXECUTE_PLANNED_PATH: giving up waiting on result");
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        cart_move_as_.setSucceeded(cart_result_); //could say "aborted"
    } else {
     * */
    ROS_INFO("finished move execution");
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_move_as_.setSucceeded(cart_result_);
    //}
    path_is_valid_ = false; // reset--require new path before next move
    busy_working_on_a_request_ = false;
    //save the last point commanded, for future reference
    std::vector <double> last_pt;
    last_pt = des_trajectory_.points.back().positions;
    int njnts = last_pt.size();
    for (int i = 0; i < njnts; i++) {
        last_arm_jnt_cmd_[i] = last_pt[i];
    }
}

//version to slow down the trajectory, stretching out time w/ factor "time_stretch_factor"

void ArmMotionInterface::rescale_planned_trajectory_time(double time_stretch_factor) {
    if (!path_is_valid_) {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        ROS_WARN("do not have a valid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }

    //given a trajectory, stretch out the arrival times by factor time_stretch_factor
    int npts = des_trajectory_.points.size();
    double arrival_time_sec, new_arrival_time_sec;
    for (int i = 0; i < npts; i++) {
        arrival_time_sec = des_trajectory_.points[i].time_from_start.toSec();
        new_arrival_time_sec = arrival_time_sec*time_stretch_factor;
        ROS_INFO("pnt %d: arrival time: %f",i,new_arrival_time_sec);
        ros::Duration arrival_duration(new_arrival_time_sec); //convert time to a ros::Duration type
        des_trajectory_.points[i].time_from_start = arrival_duration;
    }
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.computed_arrival_time = computed_arrival_time_;
    ROS_INFO("computed arrival time = %f",cart_result_.computed_arrival_time);
    cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
    cart_move_as_.setSucceeded(cart_result_);
}

//given q_start, compute the tool-flange pose, and compute a path to move delta_p with R fixed
// return the optimized joint-space path in optimal_path
// this fnc can be used fairly generally--e.g., special cases such as 20cm descent from current arm pose 
// this fnc is used within rt_arm_plan_path_current_to_goal_dp_xyz()

bool ArmMotionInterface::plan_cartesian_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p) {
    //ROS_INFO("attempting Cartesian path plan for delta-p = %f, %f, %f",delta_p_(0),delta_p_(1),delta_p_(2));
    cout << delta_p.transpose() << endl;
    // this fnc will put the optimal_path result in a global vector, accessible by main
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner_delta_p(q_start, delta_p, optimal_path_);
    if (path_is_valid_) {
        ROS_INFO("plan_cartesian_delta_p: computed valid delta-p path");
        q_vec_start_resp_ = optimal_path_.front();
        q_vec_end_resp_ = optimal_path_.back();
    } else {
        ROS_WARN("plan_cartesian_delta_p: path plan attempt not successful");
    }

    return path_is_valid_;
}

//take in q_start and q_end and build trivial path in optimal_path_ for pure joint-space move

bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start, Eigen::VectorXd q_goal) {
    ROS_INFO("setting up a joint-space path");
    path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
    if (path_is_valid_) {

        stuff_trajectory(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }
    return path_is_valid_;
}


//this is a pretty general function:
// goal contains a desired tool pose;
// path is planned from current joint state to some joint state that achieves desired tool pose

bool ArmMotionInterface::plan_path_current_to_goal_gripper_pose() {
    ROS_INFO("computing a cartesian trajectory to gripper goal pose");

    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_);
    Eigen::VectorXd q_start;
    q_start = g_q_vec_arm_Xd; 
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, goal_flange_affine_, optimal_path_);

    if (path_is_valid_) {
        stuff_trajectory(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}


bool ArmMotionInterface::plan_jspace_path_current_to_cart_gripper_pose() {
    ROS_INFO("computing a jspace trajectory to gripper goal pose");
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);  
    

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_);
    Eigen::VectorXd q_start;
    q_start = g_q_vec_arm_Xd; //get_jspace_start_(); // choose last cmd, or current joint angles
    //    bool jspace_path_planner_to_affine_goal(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);

    path_is_valid_ = cartTrajPlanner_.jspace_path_planner_to_affine_goal(q_start, goal_flange_affine_, optimal_path_);

    if (path_is_valid_) {
        stuff_trajectory(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}

//FINISH ME
bool ArmMotionInterface::plan_fine_path_current_to_goal_gripper_pose() {
    return false; 
}

bool ArmMotionInterface::plan_path_current_to_goal_dp_xyz() {
    Eigen::Vector3d dp_vec;

    ROS_INFO("called plan_path_current_to_goal_dp_xyz");
    //unpack the goal pose:
    int ndim = cart_goal_.arm_dp.size();
    if (ndim != 3) {
        ROS_WARN("requested displacement, arm_dp, is wrong dimension");
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        path_is_valid_ = false;
        return path_is_valid_;
    }
    for (int i = 0; i < 3; i++) dp_vec[i] = cart_goal_.arm_dp[i];
    ROS_INFO("requested dp = %f, %f, %f", dp_vec[0], dp_vec[1], dp_vec[2]);
    Eigen::VectorXd q_start;
    q_start = g_q_vec_arm_Xd; //get_jspace_start_(); // choose last cmd, or current joint angles    
    path_is_valid_ = plan_cartesian_delta_p(q_start, dp_vec);

    if (path_is_valid_) {
        stuff_trajectory(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::cart_moveResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10_cart_move_as");
    ros::NodeHandle nh; //standard ros node handle   
    set_ur_jnt_names();
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);
    g_q_vec_arm_Xd.resize(VECTOR_DIM);
    g_q_vec_arm_Xd[0] = 1000;
    while (g_q_vec_arm_Xd[0]>20) {
        ros::spinOnce();
        ROS_INFO("waiting for valid joint values");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("current joint values: ");
    cout<<g_q_vec_arm_Xd.transpose()<<endl;
    ROS_INFO("instantiating an ArmMotionInterface: ");
    ArmMotionInterface armMotionInterface(&nh);

    // start servicing requests:
    ROS_INFO("ready to start servicing cartesian-space goals");
    while (ros::ok()) {

        ros::spinOnce();
        ros::Duration(0.1).sleep(); //don't consume much cpu time if not actively working on a command
    }

    return 0;
}
