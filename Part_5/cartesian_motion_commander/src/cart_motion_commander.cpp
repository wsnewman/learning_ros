// cart_motion_commander:  a robot-independent library of motion commands
// wsn, Nov, 2016
#include <cartesian_motion_commander/cart_motion_commander.h>

CartMotionCommander::CartMotionCommander() :
cart_move_action_client_("cartMoveActionServer", true) { // constructor
    ROS_INFO("in constructor of CartMotionCommander");
    ROS_WARN("test warning...");

    // attempt to connect to the server:
    ROS_INFO("waiting for cartMoveActionServer: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
    got_done_callback_ = false;
}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;

void CartMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
        const arm_motion_action::arm_interfaceResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_ = *result;
    got_done_callback_ = true;
}

bool CartMotionCommander::cb_received_in_time(double max_wait_time) {
    double wait_time = 0.0;
    double dt = 0.1;
    double print_time = 0.0;
    got_done_callback_ = false;
    finished_before_timeout_ = false;
    while ((!got_done_callback_) &(wait_time < max_wait_time)) {
        wait_time += dt;
        ros::Duration(dt).sleep();
        print_time += dt;
        if (print_time > 1.0) {
            print_time -= 1.0;
            ROS_WARN("CartMotionCommander still waiting on callback");
        }
    }
    if (wait_time < max_wait_time) {
        ROS_INFO("got response in time");
        finished_before_timeout_ = true;
        return true;
    }
    else {
        ROS_WARN("did not get callback in time");
        finished_before_timeout_ = false;
        return false;
    }

}

int CartMotionCommander::execute_planned_traj(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::EXECUTE_PLANNED_TRAJ;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!cb_received_in_time(computed_arrival_time_ + 2.0)) {
        ROS_WARN("did not complete move in expected time");
        return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}


//EXECUTE_TRAJ_NSEG
int CartMotionCommander::execute_traj_nseg(int iseg) {
    ROS_INFO("requesting execution of segment %d of multi-traj plan",iseg);
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::EXECUTE_TRAJ_NSEG;
    cart_goal_.nseg = iseg;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!cb_received_in_time(MAX_WAIT_TIME)) {
        ROS_WARN("did not complete move in MAX_WAIT_TIME");
        return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

int CartMotionCommander::execute_traj_nseg(int iseg,double desired_move_time) {
    ROS_INFO("requesting execution of segment %d of multi-traj plan",iseg);
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::EXECUTE_TRAJ_NSEG;
    cart_goal_.nseg = iseg;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!cb_received_in_time(desired_move_time+2.0)) {
        ROS_WARN("did not complete move before timeout, %f sec",desired_move_time+2.0);
        return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

//Eigen::VectorXd get_joint_angles(void); 

Eigen::VectorXd CartMotionCommander::get_joint_angles(void) {
    request_q_data();
    Eigen::VectorXd angs_vecXd;
    int njnts = q_vec_.size();
    angs_vecXd.resize(njnts);
    for (int i = 0; i < njnts; i++) {
        angs_vecXd[i] = q_vec_[i];
    }
    //cout<<"angs_vecXd: "<<angs_vecXd.transpose()<<endl;
    return angs_vecXd;
}



//send goal command to request arm joint angles; these will be stored in internal variable

int CartMotionCommander::request_q_data(void) {
    ROS_INFO("requesting arm joint angles");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!cb_received_in_time(computed_arrival_time_ + 2.0)) {
        ROS_WARN("did not respond within timeout");
        return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    q_vec_ = cart_result_.q_arm;
    ROS_INFO("move returned success;  arm angles: ");
    int njnts = q_vec_.size();
    for (int ijnt = 0; ijnt < njnts; ijnt++) {
        ROS_INFO("%f", q_vec_[ijnt]);
    }
    return (int) cart_result_.return_code;
}

void CartMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::ARM_TEST_MODE;
    got_done_callback_ = false; //flag to check if got callback
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //double max_wait_time = 2.0;

    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!cb_received_in_time(3.0)) {
        ROS_WARN("giving up waiting on result");
    } else {
        ROS_INFO("finished before timeout");
        ROS_INFO("return code: %d", cart_result_.return_code);
    }
}

int CartMotionCommander::clear_multi_traj_plan(void) {

    ROS_INFO("sending command to clear multitraj vector");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::CLEAR_MULTI_TRAJ_PLAN;
    got_done_callback_ = false; //flag to check if got callback
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //double max_wait_time = 2.0;

    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!cb_received_in_time(MAX_WAIT_TIME)) {
        ROS_WARN("giving up waiting on result");
    } else {
        ROS_INFO("finished before timeout");
        ROS_INFO("return code: %d", cart_result_.return_code);
    }
}


geometry_msgs::PoseStamped CartMotionCommander::get_tool_pose_stamped(void) { // { return tool_pose_stamped_;};    
    ROS_INFO("requesting tool pose");
    geometry_msgs::PoseStamped dummy_pose;
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!cb_received_in_time(2.0)) {
        ROS_WARN("did not respond within timeout");
        return dummy_pose;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return dummy_pose;
    }

    tool_pose_stamped_ = cart_result_.current_pose_gripper;
    ROS_INFO("move returned success; tool pose: ");
    ROS_INFO("toolflange origin w/rt base = %f, %f, %f ", tool_pose_stamped_.pose.position.x,
            tool_pose_stamped_.pose.position.y, tool_pose_stamped_.pose.position.z);
    ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f", tool_pose_stamped_.pose.orientation.x,
            tool_pose_stamped_.pose.orientation.y, tool_pose_stamped_.pose.orientation.z,
            tool_pose_stamped_.pose.orientation.w);
    return tool_pose_stamped_;
}

//traj current pose to a jspace home pose

int CartMotionCommander::plan_jspace_traj_current_to_waiting_pose(int nsteps, double arrival_time) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_WAITING_POSE;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    double t_wait = 2.0; //max wait this long for planning result
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;    
}

int CartMotionCommander::plan_jspace_traj_current_to_qgoal(int nsteps, double arrival_time, Eigen::VectorXd q_goal) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_QGOAL;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    //float64[] q_goal
    cart_goal_.q_goal.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
        cart_goal_.q_goal[i] = q_goal[i];
    }
    double t_wait = 2.0; //max wait this long for planning result
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;    
}

//computes a jspace traj from start pose to some IK soln of desired tool pose

int CartMotionCommander::plan_jspace_traj_current_to_tool_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_CURRENT_TO_CART_TOOL_POSE;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    cart_goal_.des_pose_gripper = des_pose;
     double t_wait = 2.0; //max wait this long for planning result
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;    

}

int CartMotionCommander::plan_cartesian_traj_current_to_des_tool_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_CURRENT_TO_DES_TOOL_POSE;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    cart_goal_.des_pose_gripper = des_pose; 
    double t_wait = 2.0;
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;
}

int CartMotionCommander::plan_cartesian_traj_qprev_to_des_tool_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_QPREV_TO_DES_TOOL_POSE;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    cart_goal_.des_pose_gripper = des_pose; 
    double t_wait = 2.0; //max wait this long for planning result
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;    
}

int CartMotionCommander::append_multi_traj_cart_segment(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose) {
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::APPEND_MULTI_TRAJ_CART_SEGMENT;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    cart_goal_.des_pose_gripper = des_pose; 
    double t_wait = 2.0; //max wait this long for planning result
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val;      
}


int CartMotionCommander::plan_cartesian_traj_qstart_to_des_tool_pose(int nsteps, double arrival_time,
        Eigen::VectorXd q_start, geometry_msgs::PoseStamped des_pose) {

    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_CARTESIAN_TRAJ_QSTART_TO_DES_TOOL_POSE;
    cart_goal_.nsteps = nsteps; //send 10 sub-commands
    cart_goal_.arrival_time = arrival_time; //move over 2 sec
    cart_goal_.des_pose_gripper = des_pose;
    //float64[] q_start
    cart_goal_.q_start.resize(NJNTS_);
    for (int i=0;i<NJNTS_;i++) {
        cart_goal_.q_start[i] = q_start[i];
    }
    double t_wait = 2.0; //max waiting time for planning request
    int rtn_val = send_planning_goal_get_result( t_wait);
    return rtn_val; 
}

int CartMotionCommander::send_planning_goal_get_result(double t_wait) {
        cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    //ROS_INFO("return code: %d", cart_result_.return_code);
    if (!cb_received_in_time(t_wait)) {
        ROS_WARN("giving up waiting on result");
        return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code == arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code != arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    //ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}


/*
    bool plan_jspace_traj_current_to_qgoal(int nsteps, double arrival_time,Eigen::VectorXd q_goal); //traj current to a specified jspace pose
    bool plan_jspace_traj_qstart_to_qend(int nsteps, double arrival_time,Eigen::VectorXd q_start,Eigen::VectorXd q_goal);   //jspace traj from specified q_start to q_end
    bool plan_jspace_traj_current_to_tool_pose(int nsteps, double arrival_time,geometry_msgs::PoseStamped des_pose);   //computes a jspace traj from start pose to some IK soln of desired tool pose
*/



/*
void CartMotionCommander::set_arrival_time_planned_trajectory(double arrival_time) {
  ROS_INFO("scheduling arrival times in trajectory");
  computed_arrival_time_ = arrival_time;
   cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::SET_ARRIVAL_TIME_PLANNED_TRAJECTORY;
        //case arm_motion_action::arm_interfaceGoal::TIME_RESCALE_PLANNED_TRAJECTORY:
        //    time_scale_stretch_factor_ = goal->time_scale_stretch_factor;
   cart_goal_.time_scale_stretch_factor = arrival_time;
     got_done_callback_=false; //flag to check if got callback
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //double max_wait_time = 2.0;
    
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
        } else {
            ROS_INFO("finished before timeout");
            ROS_INFO("return code: %d",cart_result_.return_code);
        }         
            
}
  

void CartMotionCommander::time_rescale_planned_trajectory(double time_scale_factor) {
   ROS_INFO("rescaling speed");
   cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::TIME_RESCALE_PLANNED_TRAJECTORY;
        //case arm_motion_action::arm_interfaceGoal::TIME_RESCALE_PLANNED_TRAJECTORY:
        //    time_scale_stretch_factor_ = goal->time_scale_stretch_factor;
   cart_goal_.time_scale_stretch_factor = time_scale_factor;
     got_done_callback_=false; //flag to check if got callback
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //double max_wait_time = 2.0;
    
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
        } else {
            ROS_INFO("finished before timeout");
            ROS_INFO("return code: %d",cart_result_.return_code);
        }         
            
}
            
            

int CartMotionCommander::plan_move_to_waiting_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int CartMotionCommander::plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {    
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    int njnts = q_des_vec.size();
    cart_goal_.q_goal.resize(njnts);
    for (int i=0;i<njnts;i++) cart_goal_.q_goal[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
  
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;    
    
}

int CartMotionCommander::plan_path_current_to_goal_gripper_pose(geometry_msgs::PoseStamped des_pose) {
    
    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_PATH_CURRENT_TO_GOAL_GRIPPER_POSE;
    //ROS_INFO("des_pose: ");
    //xformUtils_.printStampedPose(des_pose);
    cart_goal_.des_pose_gripper = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;        
}

int CartMotionCommander::plan_jspace_path_current_to_cart_gripper_pose(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose_gripper) {
    ROS_WARN("requesting a joint-space motion plan to cartesian gripper pose");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_PATH_CURRENT_TO_CART_GRIPPER_POSE; 
    cart_goal_.des_pose_gripper = des_pose_gripper;
    cart_goal_.arrival_time = arrival_time;
    cart_goal_.nsteps = nsteps;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2));
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;            
}

//    bool plan_jspace_traj_qstart_to_des_tool_pose(Eigen::VectorXd  q_start,int nsteps,double arrival_time,geometry_msgs::PoseStamped des_pose);
 
bool CartMotionCommander::plan_jspace_traj_qstart_to_des_tool_pose(Eigen::VectorXd  q_start,  int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose){
     ROS_WARN("requesting a joint-space traj motion plan to cartesian gripper pose");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_TRAJ_QSTART_TO_CART_GRIPPER_POSE; 
    //geometry_msgs::PoseStamped des_pose;
    // geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);
    cart_goal_.des_pose_gripper = des_pose;
    cart_goal_.arrival_time = arrival_time;
    cart_goal_.nsteps = nsteps;
    cart_goal_.q_start.clear();
    for (int i=0;i<NJNTS_;i++) cart_goal_.q_start.push_back(q_start[i]);
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2));
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;        
}
bool CartMotionCommander::plan_jspace_path_qstart_to_cart_gripper_pose(Eigen::VectorXd q_start, int nsteps, geometry_msgs::PoseStamped des_pose) {
    ROS_WARN("requesting a joint-space motion plan to cartesian gripper pose");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_JSPACE_PATH_QSTART_TO_CART_GRIPPER_POSE; 
    cart_goal_.des_pose_gripper = des_pose;
    cart_goal_.nsteps = nsteps;
    //float64[] q_start
    cart_goal_.q_start.clear();
    for (int i=0;i<NJNTS_;i++) cart_goal_.q_start.push_back(q_start[i]);
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2));
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;           

}  


int CartMotionCommander::plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {
    
    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = arm_motion_action::arm_interfaceGoal::PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&CartMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    //finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!cb_received_in_time(2.0)) {
            ROS_WARN("giving up waiting on result");
            return (int) arm_motion_action::arm_interfaceResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==arm_motion_action::arm_interfaceResult::PATH_NOT_VALID) {
        ROS_WARN(" arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;      
}
    





*/
