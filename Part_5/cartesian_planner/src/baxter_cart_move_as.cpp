// baxter_cart_move_as: 
// wsn, April, 2016
// action server to accept commands and perform planning and motion requests
// requires trajActionServer is running (rosrun baxter_trajectory_streamer rt_arm_as)
//rosrun baxter_trajectory_streamer rt_arm_as
//rosrun baxter_trajectory_streamer left_arm_as

// move goals are specified as geometry_msgs::PoseStamped;
// it is assumed that the move goals refer to the tool frame with respect to the torso frame

//NOTE: ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE does: unpack_goal_pose(),
// which does:     a_flange_end_ = a_tool_end_*A_tool_wrt_flange_.inverse();
// then ik functions are w/rt desired flange frame

// uses library of arm-motion planning functions
#include <cartesian_planner/baxter_cartesian_planner.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
//#include <cartesian_planner/armNavSrvMsg.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include<baxter_trajectory_streamer/trajAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
//#include<baxter_traj_streamer/trajAction.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/UInt8.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/JointState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

const double ARM_ERR_TOL = 0.1; // tolerance btwn last joint commands and current arm pose
// used to decide if last command is good start point for new path

int g_right_js_doneCb_flag = 0;

class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    XformUtils xformUtils;
    //create an action server, which will be called "cartMoveActionServer"
    //this service will accept goals in Cartesian coordinates
    actionlib::SimpleActionServer<cartesian_planner::baxter_cart_moveAction> cart_move_as_;

    //also create an action client, which will send joint-space goals to the trajectory interpolator service
    // THIS ONE IS FOR RIGHT ARM ONLY
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> traj_streamer_action_client_;

    //messages to receive cartesian goals / return results:
    cartesian_planner::baxter_cart_moveGoal cart_goal_;
    cartesian_planner::baxter_cart_moveResult cart_result_;

    //messages to send goals/get results from joint-space interpolator action server:
    baxter_trajectory_streamer::trajGoal js_goal_; //goal message to send to joint-space interpolator server
    baxter_trajectory_streamer::trajResult js_result_; // server will populate this result, when done w/ goal

    //callback fnc for joint-space action server to return result to this node:
    void js_doneCb_(const actionlib::SimpleClientGoalState& state,
            const baxter_trajectory_streamer::trajResultConstPtr& result);

    //callback function to receive and act on cartesian move goal requests
    //this is the key method in this node;
    // can/should be extended to cover more motion-planning cases
    void executeCB(const actionlib::SimpleActionServer<cartesian_planner::baxter_cart_moveAction>::GoalConstPtr& goal);

    double computed_arrival_time_; //when a move time is computed, result is stored here

    //poses, from goal message:
    geometry_msgs::PoseStamped goal_gripper_pose_right_; //cmd for right-arm tool pose
    geometry_msgs::PoseStamped goal_gripper_pose_left_;

    //current tool poses w/rt torso:
    geometry_msgs::Pose current_gripper_pose_right_, current_flange_pose_right_; //cmd for right-arm tool pose
    geometry_msgs::Pose current_gripper_pose_left_, current_flange_pose_left_;
    geometry_msgs::PoseStamped current_gripper_stamped_pose_right_,current_flange_stamped_pose_right_; //cmd for right-arm tool pose
    geometry_msgs::PoseStamped current_gripper_stamped_pose_left_,current_flange_stamped_pose_left_;

    Eigen::Affine3d goal_gripper_affine_right_, goal_gripper_affine_left_;
    Eigen::Affine3d goal_flange_affine_right_, goal_flange_affine_left_;

    //have not yet implemented gripper motion commands, as anticipated in goal message
    double gripper_open_close_cmd_right_, gripper_open_close_cmd_left_; //gripper open/close commands
    unsigned short int command_mode_; //   

    Vectorq7x1 q_vec_right_arm_; //use this for current joint-space pose of robot    
    Eigen::VectorXd q_vec_right_arm_Xd_; //alt representation of above, for convenience

    Eigen::VectorXd q_start_Xd_;

    Eigen::Affine3d affine_rt_arm_tool_wrt_torso_,affine_rt_arm_flange_wrt_torso_;
    Eigen::Affine3d affine_left_arm_tool_wrt_torso_,affine_left_arm_flange_wrt_torso_;

    Eigen::Affine3d A_tool_wrt_flange_;

    double arrival_time_;
    bool path_is_valid_;

    // vec to contain optimal path from planners
    std::vector<Eigen::VectorXd> optimal_path_; //implicitly, right-arm path
    std::vector<Eigen::VectorXd> optimal_path_left_; //left-arm path
    trajectory_msgs::JointTrajectory des_trajectory_; //  trajectory object
    trajectory_msgs::JointTrajectory des_trajectory_left_; //  trajectory object for left arm
    //later, add head pan as well
    Eigen::VectorXd last_arm_jnt_cmd_right_;
    Eigen::VectorXd last_arm_jnt_cmd_left_;

    //some handy constants...
    Eigen::Matrix3d R_gripper_down_;
    Vectorq7x1 q_pre_pose_;
    Eigen::VectorXd q_pre_pose_Xd_;
    Eigen::VectorXd q_goal_pose_Xd_;

    //sensor_msgs::JointState joint_states_;
    moveit_msgs::DisplayTrajectory display_trajectory_;

    //Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver_; //instantiate a forward-kinematics solver 
    CartTrajPlanner cartTrajPlanner_; // from cartesian trajectory planner library

    Baxter_traj_streamer baxter_traj_streamer_; //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  

    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for service

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
public:
    ArmMotionInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionInterface(void) {
    }

    //handy utilities, primarily used internally, but publicly accessible
    //Eigen::Affine3d transformTFToEigen(const tf::Transform &t);
    //Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    void display_affine(Eigen::Affine3d affine);



    Eigen::VectorXd get_jspace_start_right_arm_(void); //choose between most recent cmd, or current jnt angs


    //the following methods correspond to command codes, via action message goals
    Eigen::VectorXd get_right_arm_joint_angles(void);
    //get joint angles, compute fwd kin, convert result to a stamped pose
    // put answer in current_gripper_stamped_pose_right_
    void compute_right_tool_stamped_pose(void); //helper for RT_ARM_GET_TOOL_POSE
    void compute_right_flange_stamped_pose(void); //helper for RT_ARM_GET_FLANGE_POSE
    //void compute_left_tool_stamped_pose(void); 

    // for RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE
    bool rt_arm_plan_path_current_to_goal_pose(); //uses goal.des_pose_gripper_right to plan a cartesian path
    bool rt_arm_plan_path_current_to_goal_flange_pose(); //interprets goal.des_pose_flange_right as a des FLANGE pose to plan a cartesian path
    bool rt_arm_plan_fine_path_current_to_goal_flange_pose(); //interprets goal.des_pose_flange_right as a des FLANGE pose to plan a cartesian path

    //for RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ
    bool rt_arm_plan_path_current_to_goal_dp_xyz(); //plans cartesian motion by specified 3-D displacement at fixed orientation
    bool plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p); //helper for above

    //following used in RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE
    // and RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL
    bool plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start, Vectorq7x1 q_goal);
    bool plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start_Xd, Eigen::VectorXd q_goal_Xd);
    void rescale_planned_trajectory_time(double time_stretch_factor);
    bool refine_cartesian_path_soln();

};

void ArmMotionInterface::executeCB(const actionlib::SimpleActionServer<cartesian_planner::baxter_cart_moveAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB of ArmMotionInterface");
    cart_goal_ = *goal; // copy of goal held in member var
    command_mode_ = goal->command_code;
    ROS_INFO_STREAM("received command mode " << command_mode_);
    int njnts;

    switch (command_mode_) {
        case cartesian_planner::baxter_cart_moveGoal::ARM_TEST_MODE:
            ROS_INFO("responding to request TEST_MODE: ");
            cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;
            //looks up current right-arm joint angles and returns them to client
        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_GET_Q_DATA:
            ROS_INFO("responding to request RT_ARM_GET_Q_DATA");
            get_right_arm_joint_angles(); //will update q_vec_right_arm_Xd_
            cart_result_.q_arm_right.resize(7);
            for (int i = 0; i < 7; i++) {
                cart_result_.q_arm_right[i] = q_vec_right_arm_Xd_[i];
            }
            cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE:
            ROS_INFO("responding to request RT_ARM_GET_TOOL_POSE");
            compute_right_tool_stamped_pose();
            cart_result_.current_pose_gripper_right = current_gripper_stamped_pose_right_;
            cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;
         case cartesian_planner::baxter_cart_moveGoal::RT_ARM_GET_FLANGE_POSE:
            ROS_INFO("responding to request RT_ARM_GET_FLANGE_POSE");
            compute_right_flange_stamped_pose();
            cart_result_.current_pose_flange_right = current_flange_stamped_pose_right_;
            cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;           
            //prepares a trajectory plan to move arm from current pose to pre-defined pose
        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE:
            ROS_INFO("responding to request RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE");
            q_start_Xd_ = get_jspace_start_right_arm_();
            //q_start=q_start_Xd; // convert to fixed-size vector;
            plan_jspace_path_qstart_to_qend(q_start_Xd_, q_pre_pose_Xd_);
            busy_working_on_a_request_ = false;
            break;

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL:
            ROS_INFO("responding to request RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL");
            q_start_Xd_ = get_jspace_start_right_arm_();
            q_goal_pose_Xd_.resize(7);
            njnts = goal->q_goal_right.size();
            if (njnts != 7) {
                ROS_WARN("joint-space goal is wrong dimension");
                cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
            } else {
                for (int i = 0; i < 7; i++) q_goal_pose_Xd_[i] = goal->q_goal_right[i];
                //q_start=q_start_Xd; // convert to fixed-size vector;
                plan_jspace_path_qstart_to_qend(q_start_Xd_, q_goal_pose_Xd_);
                busy_working_on_a_request_ = false;
            }
            break;
        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE:
            rt_arm_plan_path_current_to_goal_pose();
            break;
        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_FLANGE_POSE:
            rt_arm_plan_path_current_to_goal_flange_pose();
            break;

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_FINE_PATH_CURRENT_TO_GOAL_FLANGE_POSE:
            rt_arm_plan_fine_path_current_to_goal_flange_pose();
            
            break;
            
            

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ:
            rt_arm_plan_path_current_to_goal_dp_xyz();
            break;

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_REFINE_PLANNED_TRAJECTORY:
            refine_cartesian_path_soln();
            break;

        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_TIME_RESCALE_PLANNED_TRAJECTORY:
            time_scale_stretch_factor_ = goal->time_scale_stretch_factor;
            rescale_planned_trajectory_time(time_scale_stretch_factor_);
            break;

            //consults a pre-computed trajectory and invokes execution;
        case cartesian_planner::baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH: //assumes there is a valid planned path in optimal_path_
            ROS_INFO("responding to request RT_ARM_EXECUTE_PLANNED_PATH");
            execute_planned_move();
            break;

        default:
            ROS_WARN("this command mode is not defined: %d", command_mode_);
            cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::COMMAND_CODE_NOT_RECOGNIZED;
            cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }
}


//CONSTRUCTOR: pass in a node handle and perform initializations (w/ initializers)

ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle),
cart_move_as_(*nodehandle, "cartMoveActionServer", boost::bind(&ArmMotionInterface::executeCB, this, _1), false),
baxter_traj_streamer_(nodehandle),
traj_streamer_action_client_("rightArmTrajActionServer", true) { // constructor
    ROS_INFO("in class constructor of ArmMotionInterface");

    //initialize variables here, as needed
    q_pre_pose_ << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;
    q_pre_pose_Xd_ = q_pre_pose_; // copy--in VectorXd format
    q_vec_start_rqst_ = q_pre_pose_; // 0,0,0,0,0,0,0; // make this a 7-d vector
    q_vec_end_rqst_ = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_start_resp_ = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_end_resp_ = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    R_gripper_down_ = cartTrajPlanner_.get_R_gripper_down();

    //access constants defined in action message this way:
    command_mode_ = cartesian_planner::baxter_cart_moveGoal::ARM_TEST_MODE;

    received_new_request_ = false;
    busy_working_on_a_request_ = false;
    path_is_valid_ = false;
    path_id_ = 0;
    // can also do tests/waits to make sure all required services, topics, etc are alive

    A_tool_wrt_flange_ = baxter_fwd_solver_.get_affine_tool_wrt_flange();

    //check that joint-space interpolator service is connected:
    // attempt to connect to the right-arm trajectory action server:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = traj_streamer_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

    ROS_INFO("getting joint states: ");
    //q_vec_right_arm_Xd_.resize(7);
    q_vec_right_arm_Xd_ = get_right_arm_joint_angles(); //this also sets q_vec_right_arm_
    ROS_INFO("got valid right-arm joint state");
    last_arm_jnt_cmd_right_ = q_vec_right_arm_Xd_;

    ROS_INFO("starting action server: cartMoveActionServer ");
    cart_move_as_.start(); //start the server running
}

//callback fnc from joint-space trajectory streamer

void ArmMotionInterface::js_doneCb_(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO("done-callback pinged by joint-space interpolator action server done");
    g_right_js_doneCb_flag = 1;
}

//handy utility: convert from a tf::Transform object to an Eigen::Affine3d
//not used in this node; obsolete--use XformUtils instead
/*
Eigen::Affine3d ArmMotionInterface::transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}
*/
//handy utility: convert from geometry_msgs::Pose to an Eigen::Affine3d
//should instead use fncs in xformUtils package
/*
Eigen::Affine3d ArmMotionInterface::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;
    affine.translation()= Oe;
    return affine;
}
*/
//handy utility: convert from Eigen::Affine3d to a geometry_msgs::Pose
//obsolete: uses XformUtils
/*
geometry_msgs::Pose ArmMotionInterface::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}
*/
//handy utility, just to print data to screen for Affine objects

void ArmMotionInterface::display_affine(Eigen::Affine3d affine) {
    cout << "Affine origin: " << affine.translation().transpose() << endl;
    cout << "Affine rotation: " << endl;
    cout << affine.linear() << endl;
}


//right arm only--compare last command on record to current joint states
// need this to eval if last command is viable to use as start point for motion plan
// if within tolerance, return last command (for bumpless transition)
// if not within tolerance, return current joint state angles

Eigen::VectorXd ArmMotionInterface::get_jspace_start_right_arm_(void) {
    //get the current joint state
    q_vec_right_arm_Xd_ = get_right_arm_joint_angles(); //this also sets q_vec_right_arm_

    double arm_err = (last_arm_jnt_cmd_right_ - q_vec_right_arm_Xd_).norm();
    if (arm_err < ARM_ERR_TOL) {
        return last_arm_jnt_cmd_right_;
    } else {
        return q_vec_right_arm_Xd_;
    }
}

//since baxter_traj_streamer_ object already has a subscription to joint_state,
// can use it to get joint states.  Paranoid--extra tests to make sure data is "fresh"

Eigen::VectorXd ArmMotionInterface::get_right_arm_joint_angles(void) {
    //not sure this is necessary; DO want "fresh" angles, so make sure values are updated:
    q_vec_right_arm_[0] = 1000;
    while (fabs(q_vec_right_arm_[0]) > 3) { // keep trying until see viable value populated by fnc
        q_vec_right_arm_ = baxter_traj_streamer_.get_qvec_right_arm();
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    Eigen::VectorXd q_vec_xd;
    q_vec_xd = q_vec_right_arm_; // convert from 7DOF to Xd
    //while we're at it, fill in mem variable as well:
    q_vec_right_arm_Xd_ = q_vec_xd;
    return q_vec_xd;
}

//handy fnc to get current right tool pose
// gets current joint angles, does fwd kin, includes tool xform
// converts result to a geometry_msgs::PoseStamped

void ArmMotionInterface::compute_right_tool_stamped_pose(void) {
    get_right_arm_joint_angles(); //will update q_vec_right_arm_Xd_ and q_vec_right_arm_
    //fwd_kin_flange_wrt_torso_solve
    affine_rt_arm_tool_wrt_torso_ =
            baxter_fwd_solver_.fwd_kin_tool_wrt_torso_solve(q_vec_right_arm_); //rtns pose w/rt torso frame (base frame)
    current_gripper_pose_right_ = xformUtils.transformEigenAffine3dToPose(affine_rt_arm_tool_wrt_torso_);
    current_gripper_stamped_pose_right_.pose = current_gripper_pose_right_;
    current_gripper_stamped_pose_right_.header.stamp = ros::Time::now();
    current_gripper_stamped_pose_right_.header.frame_id = "torso";
}

void ArmMotionInterface::compute_right_flange_stamped_pose(void) {
    get_right_arm_joint_angles(); //will update q_vec_right_arm_Xd_ and q_vec_right_arm_
    //fwd_kin_flange_wrt_torso_solve
    affine_rt_arm_flange_wrt_torso_ =
            baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_vec_right_arm_); //rtns pose w/rt torso frame (base frame)
    current_flange_pose_right_ = xformUtils.transformEigenAffine3dToPose(affine_rt_arm_flange_wrt_torso_);
    current_flange_stamped_pose_right_.pose = current_flange_pose_right_;
    current_flange_stamped_pose_right_.header.stamp = ros::Time::now();
    current_flange_stamped_pose_right_.header.frame_id = "torso";
}

void ArmMotionInterface::execute_planned_move(void) {
    if (!path_is_valid_) {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        ROS_WARN("attempted to execute invalid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }

    // convert path to a trajectory:
    //baxter_traj_streamer_.stuff_trajectory(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
    js_goal_.trajectory = des_trajectory_;
    //computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    ROS_INFO("sending action request to traj streamer node");
    ROS_INFO("computed arrival time is %f", computed_arrival_time_);
    busy_working_on_a_request_ = true;
    g_right_js_doneCb_flag = 0;
    traj_streamer_action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::js_doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    ROS_INFO("waiting on trajectory streamer...");
    while (g_right_js_doneCb_flag == 0) {
        ROS_INFO("...");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    //finished_before_timeout_ = traj_streamer_action_client_.waitForResult(ros::Duration(computed_arrival_time_ + 2.0));
    /*
    if (!finished_before_timeout_) {
        ROS_WARN("EXECUTE_PLANNED_PATH: giving up waiting on result");
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        cart_move_as_.setSucceeded(cart_result_); //could say "aborted"
    } else {
     * */
    ROS_INFO("finished before timeout");
    cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
    cart_move_as_.setSucceeded(cart_result_);
    //}
    path_is_valid_ = false; // reset--require new path before next move
    busy_working_on_a_request_ = false;
    //save the last point commanded, for future reference
    std::vector <double> last_pt;
    last_pt = des_trajectory_.points.back().positions;
    int njnts = last_pt.size();
    for (int i = 0; i < njnts; i++) {
        last_arm_jnt_cmd_right_[i] = last_pt[i];
    }
}

//version to slow down the trajectory, stretching out time w/ factor "time_stretch_factor"
//implicitly, this is for right arm; need to create left-arm version as well

void ArmMotionInterface::rescale_planned_trajectory_time(double time_stretch_factor) {
    if (!path_is_valid_) {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        ROS_WARN("do not have a valid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }

    //given a trajectory, stretch out the arrival times by factor time_stretch_factor
    int npts = des_trajectory_.points.size();
    double arrival_time_sec, new_arrival_time_sec;
    for (int i = 0; i < npts; i++) {
        arrival_time_sec = des_trajectory_.points[i].time_from_start.toSec();
        new_arrival_time_sec = arrival_time_sec*time_stretch_factor;
        ros::Duration arrival_duration(new_arrival_time_sec); //convert time to a ros::Duration type
        des_trajectory_.points[i].time_from_start = arrival_duration;
    }
    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    cart_result_.computed_arrival_time = computed_arrival_time_;

    cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
    cart_move_as_.setSucceeded(cart_result_);
}

//given q_start, compute the tool-flange pose, and compute a path to move delta_p with R fixed
// return the optimized joint-space path in optimal_path
// this fnc can be used fairly generally--e.g., special cases such as 20cm descent from current arm pose 
// this fnc is used within rt_arm_plan_path_current_to_goal_dp_xyz()

bool ArmMotionInterface::plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p) {
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

bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start, Vectorq7x1 q_goal) {
    ROS_INFO("setting up a joint-space path");
    path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
    if (path_is_valid_) {
        //void Baxter_traj_streamer::stuff_trajectory_right_arm( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {

        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }
    return path_is_valid_;
}

//alt version: as above, but takes args of vectorXd

bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Eigen::VectorXd q_start_Xd, Eigen::VectorXd q_goal_Xd) {
    ROS_INFO("setting up a joint-space path");
    Vectorq7x1 q_start, q_goal;
    q_start = q_start_Xd; //type conversion, implicit
    q_goal = q_goal_Xd;

    path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;

}

//this is a pretty general function:
// goal contains a desired tool pose;
// path is planned from current joint state to some joint state that achieves desired tool pose

bool ArmMotionInterface::rt_arm_plan_path_current_to_goal_pose() {
    ROS_INFO("computing a cartesian trajectory to right-arm goal pose");
    //unpack the goal pose:
    goal_gripper_pose_right_ = cart_goal_.des_pose_gripper_right;

    goal_gripper_affine_right_ = xformUtils.transformPoseToEigenAffine3d(goal_gripper_pose_right_.pose);
    ROS_INFO("tool frame goal: ");
    display_affine(goal_gripper_affine_right_);
    goal_flange_affine_right_ = goal_gripper_affine_right_ * A_tool_wrt_flange_.inverse();
    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_right_);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_right_arm_(); // choose last cmd, or current joint angles
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, goal_flange_affine_right_, optimal_path_);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}

//this version uses des_pose_flange_right as desired FLANGE pose

bool ArmMotionInterface::rt_arm_plan_path_current_to_goal_flange_pose() {
    ROS_INFO("computing a cartesian trajectory to right-arm flange goal pose");
    //unpack the goal pose:
    goal_flange_affine_right_ = xformUtils.transformPoseToEigenAffine3d(cart_goal_.des_pose_flange_right.pose);

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_right_);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_right_arm_(); // choose last cmd, or current joint angles
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, goal_flange_affine_right_, optimal_path_);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}


bool ArmMotionInterface::rt_arm_plan_fine_path_current_to_goal_flange_pose() {
    ROS_INFO("computing a hi-res cartesian trajectory to right-arm flange goal pose");
    //unpack the goal pose:
    goal_flange_affine_right_ = xformUtils.transformPoseToEigenAffine3d(cart_goal_.des_pose_flange_right.pose);

    ROS_INFO("flange goal");
    display_affine(goal_flange_affine_right_);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_right_arm_(); // choose last cmd, or current joint angles
    path_is_valid_ = cartTrajPlanner_.fine_cartesian_path_planner(q_start, goal_flange_affine_right_, optimal_path_);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}

bool ArmMotionInterface::rt_arm_plan_path_current_to_goal_dp_xyz() {
    Eigen::Vector3d dp_vec;

    ROS_INFO("called rt_arm_plan_path_current_to_goal_dp_xyz");
    //unpack the goal pose:
    int ndim = cart_goal_.arm_dp_right.size();
    if (ndim != 3) {
        ROS_WARN("requested displacement, arm_dp_right, is wrong dimension");
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        path_is_valid_ = false;
        return path_is_valid_;
    }
    for (int i = 0; i < 3; i++) dp_vec[i] = cart_goal_.arm_dp_right[i];
    ROS_INFO("requested dp = %f, %f, %f", dp_vec[0], dp_vec[1], dp_vec[2]);
    Vectorq7x1 q_start;
    q_start = get_jspace_start_right_arm_(); // choose last cmd, or current joint angles    
    path_is_valid_ = plan_cartesian_delta_p(q_start, dp_vec);

    if (path_is_valid_) {
        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}

bool ArmMotionInterface::refine_cartesian_path_soln(void) {
    ROS_INFO("ArmMotionInterface: refining trajectory");
    if (path_is_valid_) {
        bool valid = cartTrajPlanner_.refine_cartesian_path_plan(optimal_path_);
        if (!valid) return false;

        baxter_traj_streamer_.stuff_trajectory_right_arm(optimal_path_, des_trajectory_); //convert from vector of poses to trajectory message   
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_cart_move_as");
    ros::NodeHandle nh; //standard ros node handle   

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
