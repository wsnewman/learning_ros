void ArmMotionInterface::executeCB(const actionlib::SimpleActionServer<arm_motion_action::arm_interfaceAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB of ArmMotionInterface");
    cart_goal_ = *goal; // copy of goal held in member var
    command_mode_ = goal->command_code;
    ROS_INFO_STREAM("received command mode " << command_mode_);
    int njnts;

    switch (command_mode_) {
        //a simple "is-alive" test
        case arm_motion_action::arm_interfaceGoal::ARM_TEST_MODE:
            ROS_INFO("responding to request TEST_MODE: ");
            cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;
        //a couple of queries:
        case arm_motion_action::arm_interfaceGoal::GET_TOOL_POSE:
            ROS_INFO("responding to request GET_TOOL_POSE");
            compute_tool_stamped_pose();
            cart_result_.current_pose_gripper = current_gripper_stamped_pose_;
            cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;
        //looks up current arm joint angles and returns them to client
        case arm_motion_action::arm_interfaceGoal::GET_Q_DATA:
            ROS_INFO("responding to request GET_Q_DATA");
            //get_joint_angles(); 
            cart_result_.q_arm.resize(NJNTS_);
            for (int i = 0; i < NJNTS_; i++) {
                cart_result_.q_arm[i] = q_vec_arm_Xd_[i];
            }
            cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;  
 
        //this cmd checks if have a valid pre-computed trajectory, and if so, invokes execution;
        case arm_motion_action::arm_interfaceGoal::EXECUTE_PLANNED_TRAJ: //assumes there is a valid planned path in optimal_path_
            ROS_INFO("responding to request EXECUTE_PLANNED_TRAJ");
            execute_planned_traj(); //this fnc does setSucceeded on its own
            break;

        //this version executes 1 of NSEG segments of a vector of trajectories
        case arm_motion_action::arm_interfaceGoal::EXECUTE_TRAJ_NSEG: //assumes there is a valid planned path in optimal_path_
            ROS_INFO("responding to request EXECUTE_PLANNED_TRAJ");
            execute_traj_nseg(); //this fnc does setSucceeded on its own
            break;            
            
            
        //various flavors of trajectory planning:
            
        //prepares a trajectory plan to move arm from current pose to pre-defined pose
        case arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_WAITING_POSE:
            ROS_INFO("responding to request PLAN_TRAJ_CURRENT_TO_WAITING_POSE");
            plan_jspace_traj_current_to_waiting_pose(); //q_start_Xd_, q_pre_pose_Xd_);
            busy_working_on_a_request_ = false;
            break;   
            
        //prepares a jspace trajectory plan to move arm from current pose to specified goal pose
        case arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_QGOAL:
            ROS_INFO("responding to request PLAN_JSPACE_TRAJ_CURRENT_TO_QGOAL");
            plan_jspace_traj_current_to_qgoal(); //q_start_Xd_, q_pre_pose_Xd_);
            busy_working_on_a_request_ = false;
            break;   
            
        case arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_CART_TOOL_POSE:
            ROS_INFO("responding to request PLAN_JSPACE_TRAJ_CURRENT_TO_CART_TOOL_POSE");
            plan_jspace_traj_current_to_tool_pose(); //q_start_Xd_, q_pre_pose_Xd_);
            busy_working_on_a_request_ = false;
            break;  
            
            
        case arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_CURRENT_TO_DES_TOOL_POSE:
            ROS_INFO("responding to request PLAN_CARTESIAN_TRAJ_CURRENT_TO_DES_TOOL_POSE");
            plan_cartesian_traj_current_to_des_tool_pose();
            break;
            
        case arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_QSTART_TO_DES_TOOL_POSE:
            ROS_INFO("responding to request PLAN_CARTESIAN_TRAJ_QSTART_TO_DES_TOOL_POSE");
            plan_cartesian_traj_qstart_to_des_tool_pose();
            break;

        case arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_QPREV_TO_DES_TOOL_POSE:
            ROS_INFO("responding to request PLAN_CARTESIAN_TRAJ_QPREV_TO_DES_TOOL_POSE");
            plan_cartesian_traj_qprev_to_des_tool_pose();
            break;    
            
            //uint8 CLEAR_MULTI_TRAJ_PLAN = 26
//uint8 APPEND_MULTI_TRAJ_CART_SEGMENT = 27
//uint8 APPEND_MULTI_TRAJ_JSPACE_SEGMENT = 28
        case arm_motion_action::arm_interfaceGoal::CLEAR_MULTI_TRAJ_PLAN:
            ROS_INFO("responding to request CLEAR_MULTI_TRAJ_PLAN");
            multi_traj_vec_.clear();
            cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
            cart_move_as_.setSucceeded(cart_result_);
            break;   
        case arm_motion_action::arm_interfaceGoal::APPEND_MULTI_TRAJ_CART_SEGMENT:
            ROS_INFO("responding to request APPEND_MULTI_TRAJ_CART_SEGMENT");
            append_multi_traj_cart_segment();
            break;   
        case arm_motion_action::arm_interfaceGoal::APPEND_MULTI_TRAJ_JSPACE_SEGMENT:
            ROS_INFO("responding to request APPEND_MULTI_TRAJ_JSPACE_SEGMENT");
            append_multi_traj_jspace_segment();
            break;               
    
        default:
            ROS_WARN("this command mode is not defined: %d", command_mode_);
            cart_result_.return_code = arm_motion_action::arm_interfaceResult::COMMAND_CODE_NOT_RECOGNIZED;
            cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }
}

//END OF SWITCH/CASE STATEMENTS
