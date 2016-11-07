//ArmMotionCommander library for UR10.  Defines class useful for communicating w/ cartMoveActionServer

#include <cartesian_planner/ur10_arm_motion_commander.h>


ArmMotionCommander::ArmMotionCommander(ros::NodeHandle* nodehandle) : nh_(*nodehandle),
cart_move_action_client_("cart_move_action_server", true) { // constructor
    ROS_INFO("in constructor of ArmMotionInterface");
    q_vec_.resize(NJNTS);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 

}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;

void ArmMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
        const cartesian_planner::ur10_cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_ = *result;
}

void ArmMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
    } else {
        ROS_INFO("finished before timeout");
        ROS_INFO("return code: %d", cart_result_.return_code);
    }
}

int ArmMotionCommander::plan_move_to_pre_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    cart_goal_.q_goal.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) cart_goal_.q_goal[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;

}

//plans a joint-space path from current pose to some IK soln of cartesian goal pose of tool flange
int ArmMotionCommander::plan_jspace_path_current_to_flange_pose(geometry_msgs::PoseStamped des_pose) {
    ROS_INFO("ArmMotionCommander requesting a joint-space motion plan to Cartesian goal");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_CART_POSE;
    cart_goal_.des_pose_flange = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d", cart_result_.return_code);   
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
   ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
    
}

int ArmMotionCommander::plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose) {

    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_flange = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}

// bool ArmMotionInterface::jspace_path_planner_current_to_affine_goal(Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path)

//same as above, but assumes tool-flange pose specification, not gripper-finger-frame specification

int ArmMotionCommander::plan_path_current_to_goal_flange_pose(geometry_msgs::PoseStamped des_pose) {

    ROS_INFO("requesting a cartesian-space motion plan w/ flange pose goal");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_FLANGE_POSE;
    cart_goal_.des_pose_flange = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(5.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::plan_fine_path_current_to_goal_flange_pose(geometry_msgs::PoseStamped des_pose) {

    ROS_INFO("requesting a hi-res cartesian-space motion plan w/ flange pose goal");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_FINE_PATH_CURRENT_TO_GOAL_FLANGE_POSE;
    cart_goal_.des_pose_flange = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(5.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {

    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp.resize(3);
    for (int i = 0; i < 3; i++) cart_goal_.arm_dp[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d", cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == cartesian_planner::ur10_cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::execute_planned_path(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_ + 6.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::timestretch_planned_path(double time_stretch_factor) {
    ROS_INFO("requesting path time-stretch with factor %f", time_stretch_factor);
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::TIME_RESCALE_PLANNED_TRAJECTORY;
    cart_goal_.time_scale_stretch_factor = time_stretch_factor;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("this is taking too long...");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("request did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("request returned success");

    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f", computed_arrival_time_);
    return (int) cart_result_.return_code;
}


//send goal command to request joint angles; these will be stored in internal variable

int ArmMotionCommander::request_q_data(void) {
    ROS_INFO("requesting joint angles");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_ + 2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    q_vec_ = cart_result_.q_arm;
    ROS_INFO("move returned success;  arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f", q_vec_[0], q_vec_[1], q_vec_[2], q_vec_[3], q_vec_[4], q_vec_[5]);
    return (int) cart_result_.return_code;
}

Eigen::VectorXd ArmMotionCommander::get_joint_angles(void) {
    request_q_data();
    Eigen::VectorXd angs_vecXd;
    angs_vecXd.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) {
        angs_vecXd[i] = q_vec_[i];
    }
    return angs_vecXd;
}

int ArmMotionCommander::request_tool_pose_wrt_base(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo base_link tool0...unless have a tool frame
    ROS_INFO("requesting tool pose");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    tool_pose_stamped_ = cart_result_.current_pose_gripper;
    ROS_INFO("move returned success; tool pose: ");
    ROS_INFO("origin w/rt base = %f, %f, %f ", tool_pose_stamped_.pose.position.x,
            tool_pose_stamped_.pose.position.y, tool_pose_stamped_.pose.position.z);
    ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f", tool_pose_stamped_.pose.orientation.x,
            tool_pose_stamped_.pose.orientation.y, tool_pose_stamped_.pose.orientation.z,
            tool_pose_stamped_.pose.orientation.w);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::request_flange_pose_wrt_base(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo base_link tool0
    ROS_INFO("requesting flange pose");
    cart_goal_.command_code = cartesian_planner::ur10_cart_moveGoal::GET_FLANGE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::ur10_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != cartesian_planner::ur10_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    flange_pose_stamped_ = cart_result_.current_pose_flange;
    ROS_INFO("move returned success; flange pose: ");
    ROS_INFO("origin w/rt torso = %f, %f, %f ", flange_pose_stamped_.pose.position.x,
            flange_pose_stamped_.pose.position.y, flange_pose_stamped_.pose.position.z);
    ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f", flange_pose_stamped_.pose.orientation.x,
            flange_pose_stamped_.pose.orientation.y, flange_pose_stamped_.pose.orientation.z,
            flange_pose_stamped_.pose.orientation.w);
    return (int) cart_result_.return_code;
}
