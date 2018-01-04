// arm_motion_interface.cpp: 
// wsn,  Dec, 2017
// implementaton for ArmMotionInterface, used in the action server

#include<arm_motion_interface/arm_motion_interface.h>
#include "arm_motion_interface_switcher.cpp"

ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle, ArmMotionInterfaceInits armMotionInterfaceInits) : nh_(*nodehandle),
cart_move_as_(*nodehandle, "cartMoveActionServer", boost::bind(&ArmMotionInterface::executeCB, this, _1), false), action_client_(armMotionInterfaceInits.traj_as_name.c_str(), true)
//traj_publisher_("joint_path_command", 1)
{ // constructor
    min_dt_traj_ = 0.005; // minimum time
    ROS_INFO("in class constructor of ArmMotionInterface");
    int njnts = armMotionInterfaceInits.jnt_names.size(); // set and remember the number of joints for this robot
    NJNTS_ = njnts;
    cout << "joint names: " << endl;
    for (int i = 0; i < njnts; i++) {
        jnt_names_.push_back(armMotionInterfaceInits.jnt_names[i]);
        cout << jnt_names_[i] << " ";
    }
    cout << endl;
    for (int i=0; i< njnts; i++) {
        js_goal_.trajectory.joint_names.push_back(jnt_names_[i]);
    }

    //unpack the rest of the initializations:
    q_lower_limits_.resize(njnts);
    q_upper_limits_.resize(njnts);
    qdot_max_vec_.resize(njnts);
    q_home_pose_.resize(njnts);

    urdf_base_frame_name_ = armMotionInterfaceInits.urdf_base_frame_name;
    urdf_flange_frame_name_ = armMotionInterfaceInits.urdf_flange_frame_name;
    joint_states_topic_name_ = armMotionInterfaceInits.joint_states_topic_name;
    traj_pub_topic_name_ = armMotionInterfaceInits.traj_pub_topic_name;
    traj_as_name_ = armMotionInterfaceInits.traj_as_name;
    use_trajectory_action_server_ = armMotionInterfaceInits.use_trajectory_action_server; //           


    // armMotionInterfaceInits.urdf_base_frame_name = g_urdf_base_frame_name;

    for (int i = 0; i < njnts; i++) {
        q_lower_limits_[i] = armMotionInterfaceInits.q_lower_limits[i];
        q_upper_limits_[i] = armMotionInterfaceInits.q_upper_limits[i];
        qdot_max_vec_[i] = armMotionInterfaceInits.qdot_max_vec[i];
        q_home_pose_[i] = armMotionInterfaceInits.q_home_pose[i];

    }
    q_pre_pose_Xd_ = q_home_pose_; // synonym; but could define alternative pre-pose;

    q_vec_arm_Xd_.resize(njnts);

    pIKSolver_ = armMotionInterfaceInits.pIKSolver_arg;
    pFwdSolver_ = armMotionInterfaceInits.pFwdSolver_arg;

    //TEST TEST TEST
    ROS_INFO("arm_motion_interface, testing fk pointer...");
    Eigen::VectorXd q_vec;
    q_vec.resize(6);
    q_vec << 0, 0, 0, 0, 0, 0;
    //Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec)
    Eigen::Affine3d test_affine;
    test_affine = pFwdSolver_->fwd_kin_solve(q_vec);
    std::cout << "fwd kin of home pose: origin = " << test_affine.translation().transpose() << std::endl;

    //pointer to CartTrajPlanner obj w/ proper pointers to IK and FK solvers
    pCartTrajPlanner_ = new CartTrajPlanner(armMotionInterfaceInits.pIKSolver_arg, armMotionInterfaceInits.pFwdSolver_arg, njnts);
    pCartTrajPlanner_->set_jspace_planner_weights(armMotionInterfaceInits.planner_joint_weights);
    //set_joint_names(vector<string> jnt_names);
    pCartTrajPlanner_->set_joint_names(jnt_names_); //inform cartTrajPlanner of joint names
    /*  

    //initialize variables here, as needed
    q_pre_pose_Xd_.resize(NJNTS_);
    q_pre_pose_Xd_ << 0, 0, 0, 0, 0, 0;
    //q_pre_pose_Xd_ = q_pre_pose_; // copy--in VectorXd format
    q_vec_start_rqst_.resize(NJNTS_); // = q_pre_pose_; // 0,0,0,0,0,0,0; // make this a N-d vector
    q_vec_end_rqst_.resize(NJNTS_); // = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_start_resp_.resize(NJNTS_);// = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    q_vec_end_resp_.resize(NJNTS_);// = q_pre_pose_; //<< 0,0,0,0,0,0,0;
    R_gripper_down_ = pCartTrajPlanner_->get_R_gripper_down();
     */
    //access constants defined in action message this way:
    command_mode_ = arm_motion_action::arm_interfaceGoal::ARM_TEST_MODE;
    path_is_valid_ = false;
    traj_is_valid_ = false;
    //received_new_request_ = false;
    busy_working_on_a_request_ = false;

    //path_id_ = 0;
    // can also do tests/waits to make sure all required services, topics, etc are alive

    tfListener_ = new tf::TransformListener; //create a transform listener and assign its pointer
    bool tferr = true;
    int ntries = 0;
    ROS_INFO("waiting for tf between generic gripper frame and tool flange...");

    while (tferr) {
        tferr = false;
        try {
            //edit 12/10/17: compute transform to flange_frame instead of link7
            tfListener_->lookupTransform("generic_gripper_frame", armMotionInterfaceInits.urdf_flange_frame_name.c_str(), ros::Time(0), generic_toolflange_frame_wrt_gripper_frame_stf_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5) ROS_WARN("did you launch robot's static_transforms.launch?");
        }
    }
    ROS_INFO("tf is good for generic gripper frame w/rt tool flange");
    xformUtils.printStampedTf(generic_toolflange_frame_wrt_gripper_frame_stf_);

    //   Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::PoseStamped stPose); 
    //transformPoseToEigenAffine3d
    affine_tool_wrt_flange_ = xformUtils.transformStampedTfToEigenAffine3d(generic_toolflange_frame_wrt_gripper_frame_stf_);
    affine_flange_wrt_tool_ = affine_tool_wrt_flange_.inverse();
    tferr = true;
    ROS_INFO("waiting for tf between system_ref_frame and base_link...");

    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("system_ref_frame", armMotionInterfaceInits.urdf_base_frame_name.c_str(), ros::Time(0), base_link_wrt_system_ref_frame_stf_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good for system_ref_frame and base_link");
    xformUtils.printStampedTf(base_link_wrt_system_ref_frame_stf_);

    ROS_INFO("waiting for tf between base_link and world...");
    tferr = true;
    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("world", armMotionInterfaceInits.urdf_base_frame_name.c_str(), ros::Time(0), base_link_wrt_world_stf_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good for world and base_link");
    xformUtils.printStampedTf(base_link_wrt_world_stf_);

    //establish publisher to command joints; assumes interface is simple publish/subscribe
    traj_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>(armMotionInterfaceInits.traj_pub_topic_name.c_str(), 1, true);

 

    ROS_WARN("setting up joint-states subscriber...");
    int size_q_vec_arm_Xd = q_vec_arm_Xd_.size();
    ROS_INFO("size of q_vec_arm_Xd_ = %d",size_q_vec_arm_Xd);
    joint_states_subscriber_ = nh_.subscribe(armMotionInterfaceInits.joint_states_topic_name.c_str(), 1, &ArmMotionInterface::jointStatesCb, this);
    q_vec_arm_Xd_[0] = 1000; // impossible value; wait for it to change
    ROS_INFO("getting joint states... ");
    while (q_vec_arm_Xd_[0] > 900) {
        ROS_INFO("trying...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got joint states: ");
    cout << q_vec_arm_Xd_.transpose() << endl;

    //
    //    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    //        arm_action_client("/arm_controller/follow_joint_trajectory", true);
            //TEST TEST TEST--try manual naming
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client(traj_as_name_.c_str(), true);
 
    if (!use_trajectory_action_server_) ROS_WARN("using trajectory publisher");
    if (use_trajectory_action_server_) {
        ROS_WARN("attempting to connect to trajectory action server");
        cout<<traj_as_name_.c_str()<<endl;

        ROS_INFO("waiting for arm-control server: ");
        bool server_exists = action_client_.waitForServer(ros::Duration(1.0));
        while (!server_exists) {
            ROS_WARN("waiting on arm server...");
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            server_exists = action_client_.waitForServer(ros::Duration(1.0));
        }
        ROS_INFO("connected to arm action server"); // if here, then we connected to the server; 

    }
    //

    ROS_INFO("commanding arm to go to current pose: ");
    optimal_path_.clear();

    optimal_path_.push_back(q_vec_arm_Xd_); //(q_vec_arm_Xd_);
    optimal_path_.push_back(q_vec_arm_Xd_); //(q_vec_arm_Xd_);
    stuff_trajectory(optimal_path_, des_trajectory_);
    traj_publisher_.publish(des_trajectory_);
    ros::Duration(1.0).sleep();
    ROS_INFO("sending again: ");
    traj_publisher_.publish(des_trajectory_);
    ros::Duration(1.0).sleep();

    ROS_INFO("sending arm to home position: ");
    optimal_path_[1] = q_home_pose_;
    stuff_trajectory(optimal_path_, des_trajectory_);
    if (!use_trajectory_action_server_) { // use publisher, not action client
        traj_publisher_.publish(des_trajectory_);
        ros::Duration(1.0).sleep(); // no feedback from subscriber, so simply wait for move time
        ROS_INFO("published trajectory command");
        traj_is_valid_ = false; // reset--require new path before next move
    } else {
        //use action-server interface instead
        js_goal_.trajectory = des_trajectory_;
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        ROS_INFO("sending action request");
        ROS_INFO("computed arrival time is %f", computed_arrival_time_);
        busy_working_on_a_request_ = true;
        arm_motion_doneCb_flag_ = false;
        //arm_action_client.sendGoal(goal, &armDoneCb);
        action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::armDoneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
        ROS_INFO("waiting on trajectory streamer...");
        while (!arm_motion_doneCb_flag_) {
            ROS_INFO("...");
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            cout << "jnt angs: " << q_vec_arm_Xd_.transpose() << endl;
        }     
        traj_is_valid_ = false;
    }       
    //wait to settle:
    ros::Duration(2.0).sleep();
    //get jnt angles
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    cout << "jnt vals: " << q_vec_arm_Xd_.transpose() << endl;

    test_affine = pFwdSolver_->fwd_kin_solve(q_vec_arm_Xd_); //tool-flange frame
    std::cout << "fwd kin of home pose: origin = " << test_affine.translation().transpose() << std::endl;


    ROS_INFO("starting action server: cartMoveActionServer ");
    cart_move_as_.start(); //start the server running
}

// * void joint_states_CB(const sensor_msgs::JointState& js_msg)

void ArmMotionInterface::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    int nsize_indices = arm_joint_indices_.size();
    //ROS_INFO("nsize_indices  is %d",nsize_indices);
    if (nsize_indices < 1) {
        /*
        int njnts = js_msg.position.size();
        if (njnts != NJNTS_) {
            ROS_WARN("mismatch in number of joints; quitting");
            exit(1);
        }
         * */
        ROS_INFO("finding joint mappings"); // for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
        cout<<"resulting joint index mapping: "<<endl;
        for (int i=0;i<NJNTS_;i++) {
            cout<<arm_joint_indices_[i]<<",";
        }
        cout<<endl;
    }
    // get joint-angle values from message and copy to private variable
    for (int i = 0; i < NJNTS_; i++) {
        q_vec_arm_Xd_[i] = js_msg.position[arm_joint_indices_[i]];
    }
}

//callback fnc from joint-space trajectory streamer
//action server will respond to this callback when done

void ArmMotionInterface::armDoneCb_(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    arm_motion_doneCb_flag_ = true;

}

//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message

void ArmMotionInterface::map_arm_joint_indices(vector<string> joint_names_arg) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;
    int joint_states_dim = joint_names_arg.size();

    arm_joint_indices_.clear();
    int index;
    int njnts = jnt_names_.size(); //private variable, filled during construction

    cout<<"num jnt names = "<<njnts<<endl;
    std::string j_name;

    for (int j = 0; j < njnts; j++) {
        j_name = jnt_names_[j]; //known name, in preferred order
        cout<<"jnt name: "<<j_name<<endl;
        for (int i = 0; i < joint_states_dim; i++) {
            if (j_name.compare(joint_names_arg[i]) == 0) {
                index = i;
                cout<<"found match at index = "<<i<<endl;
                arm_joint_indices_.push_back(index);
                break;
            }
        }
    }
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by constructor

void ArmMotionInterface::stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    trajectory_point.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < NJNTS_; i++) {
        new_trajectory.joint_names.push_back(jnt_names_[i].c_str());
    }

    //try imposing a time delay on first point to work around ros_controller complaints
    double t_start = 0.001; //0.05;

    new_trajectory.header.stamp = ros::Time::now(); //+ros::Duration(t_start);  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = t_start;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout << "stuff_traj: start pt = " << q_start.transpose() << endl;
    ROS_INFO("stuffing trajectory");

    trajectory_point.positions.clear();
    trajectory_point.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < NJNTS_; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        cout << "dqvec: " << dqvec.transpose() << endl;
        del_time = min_transition_time(dqvec);
        if (del_time < min_dt_traj_)
            del_time = min_dt_traj_;
        cout << "stuff_traj: next pt = " << q_end.transpose() << endl;
        net_time += del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f", iq, del_time, net_time);
        for (int i = 0; i < NJNTS_; i++) { //copy over the joint-command values
            trajectory_point.positions[i] = q_end[i];
        }
        trajectory_point.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point);
    }
    //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout << "traj pt: ";
        for (int j = 0; j < NJNTS_; j++) {
            cout << new_trajectory.points[iq].positions[j] << ", ";
        }
        cout << endl;
        cout << "arrival time: " << new_trajectory.points[iq].time_from_start.toSec() << endl;
    }
    new_trajectory.header.stamp = ros::Time::now();;
}

double ArmMotionInterface::min_transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / qdot_max_vec_[0];
    double ti;
    for (int i = 1; i < NJNTS_; i++) {
        ti = fabs(dqvec[i]) / qdot_max_vec_[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}


//implementations of armMotionInterface member functions
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void ArmMotionInterface::execute_planned_traj(void) {
    if (!traj_is_valid_) {
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        ROS_WARN("attempted to execute invalid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }

    // convert path to a trajectory:
    //stuff_trajectory(optimal_path_, des_trajectory_);
    des_trajectory_.header.stamp = ros::Time::now();
    ROS_INFO("sending trajectory");
    ROS_INFO("computed arrival time is %f", computed_arrival_time_);

    //the following is for a publish/subscribe ROS-I interface, not action server interface to robot
    if (!use_trajectory_action_server_) { // use publisher, not action client
        traj_publisher_.publish(des_trajectory_);
        ros::Duration(computed_arrival_time_).sleep(); // no feedback from subscriber, so simply wait for move time
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
        cart_move_as_.setSucceeded(cart_result_);
        ROS_INFO("published trajectory command");
        traj_is_valid_ = false; // reset--require new path before next move
    } else {
        //use action-server interface instead
        js_goal_.trajectory = des_trajectory_;
        //computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        ROS_INFO("sending action request");
        ROS_INFO("computed arrival time is %f", computed_arrival_time_);
        busy_working_on_a_request_ = true;
        arm_motion_doneCb_flag_ = false;
        //arm_action_client.sendGoal(goal, &armDoneCb);
        action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::armDoneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
        ROS_INFO("waiting on trajectory streamer...");
        while (!arm_motion_doneCb_flag_) {
            ROS_INFO("...");
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            cout << "jnt angs: " << q_vec_arm_Xd_.transpose() << endl;
        }
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
        cart_move_as_.setSucceeded(cart_result_);        
        traj_is_valid_ = false;
    }
}

void ArmMotionInterface::execute_traj_nseg() {
    trajectory_msgs::JointTrajectoryPoint trajectory_point_last;
    int nsegs_tot = multi_traj_vec_.size();
    int nseg = cart_goal_.nseg;
    if (nseg > nsegs_tot) {
        ROS_WARN("requested traj segment beyond end of vector");
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        ROS_WARN("attempted to execute invalid path!");
        cart_move_as_.setAborted(cart_result_); // tell the client we have given up on this goal; send the result message as well
    }
    des_trajectory_ = multi_traj_vec_[nseg];
    des_trajectory_.header.stamp = ros::Time::now();

    computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
    
    if (!use_trajectory_action_server_) { // use publisher, not action client
        traj_publisher_.publish(des_trajectory_);
        ros::Duration(computed_arrival_time_).sleep(); // no feedback from subscriber, so simply wait for move time
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
        cart_move_as_.setSucceeded(cart_result_);
        ROS_INFO("published trajectory command");
        traj_is_valid_ = false; // reset--require new path before next move
    } else {
        //use action-server interface instead
        js_goal_.trajectory = des_trajectory_;
        //computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        ROS_INFO("sending action request");
        ROS_INFO("computed arrival time is %f", computed_arrival_time_);
        busy_working_on_a_request_ = true;
        arm_motion_doneCb_flag_ = false;
        //arm_action_client.sendGoal(goal, &armDoneCb);
        action_client_.sendGoal(js_goal_, boost::bind(&ArmMotionInterface::armDoneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
        ROS_INFO("waiting on trajectory streamer...");
        while (!arm_motion_doneCb_flag_) {
            ROS_INFO("...");
            ros::Duration(0.25).sleep();
            ros::spinOnce();
            cout << "jnt angs: " << q_vec_arm_Xd_.transpose() << endl;
        }
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
         traj_is_valid_ = false;
        ROS_INFO("completed nseg execution");       
        cart_move_as_.setSucceeded(cart_result_);        
    }
}

//this fnc assumes user has provided nsteps and arrival time in goal message

bool ArmMotionInterface::plan_jspace_traj_current_to_waiting_pose() {
    int nsteps = cart_goal_.nsteps;
    double arrival_time = cart_goal_.arrival_time;
    //invoke general joint-space planner fnc; specify q_start = q_current and q_goal in home pose;
    //set trajectory arg to member var des_trajectory_
    //set member var traj_is_valid_ to result of plan
    traj_is_valid_ = pCartTrajPlanner_->plan_jspace_traj_qstart_to_qend(q_vec_arm_Xd_, q_home_pose_, nsteps, arrival_time, des_trajectory_);
    if (traj_is_valid_) multi_traj_vec_.push_back(des_trajectory_segment_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::plan_jspace_traj_current_to_qgoal() {

    int njnts = cart_goal_.q_goal.size();
    if (njnts != NJNTS_) {
        ROS_WARN("goal message does not have valid q_goal");
        traj_is_valid_ = false;
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
        return false;
    }

    //get goal pose from goal message:
    q_goal_pose_Xd_.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
        q_goal_pose_Xd_[i] = cart_goal_.q_goal[i];
    }
    int nsteps = cart_goal_.nsteps;
    double arrival_time = cart_goal_.arrival_time;

    //invoke general joint-space planner fnc; specify q_start = q_current and q_goal 
    //set trajectory arg to member var des_trajectory_
    //set member var traj_is_valid_ to result of plan
    traj_is_valid_ = pCartTrajPlanner_->plan_jspace_traj_qstart_to_qend(q_vec_arm_Xd_, q_goal_pose_Xd_, nsteps, arrival_time, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::plan_jspace_traj_current_to_tool_pose() {
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;

    traj_is_valid_ = pCartTrajPlanner_->plan_jspace_traj_qstart_to_affine_goal(q_vec_arm_Xd_, goal_flange_affine_, nsteps_, arrival_time_, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::append_multi_traj_cart_segment() {
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;
    //pick off last point of previous trajectory
    int n_traj_segs = multi_traj_vec_.size();
    if (n_traj_segs < 1) {
        ROS_WARN("previous trajectory invalid");
        traj_is_valid_ = false;
        traj_plan_wrapup();
        return traj_is_valid_;
    }
    //previous traj is valid, so pick off last point:
    trajectory_msgs::JointTrajectoryPoint trajectory_point = (multi_traj_vec_.back()).points.back();
    //use this point as start of next trajectory
    for (int i = 0; i < NJNTS_; i++) { //set start jspace pose
        q_vec_arm_Xd_[i] = trajectory_point.positions[i]; //
    }
    traj_is_valid_ = pCartTrajPlanner_->plan_cartesian_traj_qstart_to_des_flange_affine(q_vec_arm_Xd_, goal_flange_affine_, nsteps_, arrival_time_, des_trajectory_segment_);
    if (traj_is_valid_) multi_traj_vec_.push_back(des_trajectory_segment_);
    traj_plan_wrapup();
    return traj_is_valid_;

}

bool ArmMotionInterface::append_multi_traj_jspace_segment() {
    ROS_WARN("append_multi_traj_jspace_segment not implemented yet");
    traj_is_valid_ = false;
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::plan_cartesian_traj_qprev_to_des_tool_pose() {
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;
    //pick off last point of previous trajectory
    int n_traj_segs = multi_traj_vec_.size();
    if (n_traj_segs < 1) {
        ROS_WARN("previous trajectory invalid");
        traj_is_valid_ = false;
        traj_plan_wrapup();
        return traj_is_valid_;
    }
    //previous traj is valid, so pick off last point:
    trajectory_msgs::JointTrajectoryPoint trajectory_point = (multi_traj_vec_.back()).points.back();
    //use this point as start of next trajectory
    for (int i = 0; i < NJNTS_; i++) { //set start jspace pose
        q_vec_arm_Xd_[i] = trajectory_point.positions[i]; //
    }

    traj_is_valid_ = pCartTrajPlanner_->plan_cartesian_traj_qstart_to_des_flange_affine(q_vec_arm_Xd_, goal_flange_affine_, nsteps_, arrival_time_, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::plan_cartesian_traj_current_to_des_tool_pose() {
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;
    traj_is_valid_ = pCartTrajPlanner_->plan_cartesian_traj_qstart_to_des_flange_affine(q_vec_arm_Xd_, goal_flange_affine_, nsteps_, arrival_time_, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;

}

bool ArmMotionInterface::plan_cartesian_traj_qstart_to_des_tool_pose() {
    goal_gripper_pose_ = cart_goal_.des_pose_gripper;
    xformUtils.printStampedPose(goal_gripper_pose_);
    goal_flange_affine_ = xform_gripper_pose_to_affine_flange_wrt_base(goal_gripper_pose_);
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;
    q_vec_start_rqst_.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
        q_vec_start_rqst_[i] = cart_goal_.q_start[i];
    }
    traj_is_valid_ = pCartTrajPlanner_->plan_cartesian_traj_qstart_to_des_flange_affine(q_vec_start_rqst_, goal_flange_affine_, nsteps_, arrival_time_, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

bool ArmMotionInterface::plan_jspace_traj_qstart_to_qend() {
    int njnts = cart_goal_.q_goal.size();
    if (njnts != NJNTS_) {
        ROS_WARN("goal message does not have valid q_goal");
        traj_is_valid_ = false;
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
        return traj_is_valid_;
    }
    //also check start qvec:
    njnts = cart_goal_.q_start.size();
    if (njnts != NJNTS_) {
        ROS_WARN("goal message does not have valid q_start");
        traj_is_valid_ = false;
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
        return traj_is_valid_;
    }

    //get start and goal poses from goal message:
    q_goal_pose_Xd_.resize(NJNTS_);
    q_vec_start_rqst_.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
        q_goal_pose_Xd_[i] = cart_goal_.q_goal[i];
        q_vec_start_rqst_[i] = cart_goal_.q_start[i];
    }
    //user must also set nsteps and arrival time:
    arrival_time_ = cart_goal_.arrival_time;
    nsteps_ = cart_goal_.nsteps;


    traj_is_valid_ = pCartTrajPlanner_->plan_jspace_traj_qstart_to_qend(q_vec_start_rqst_, q_goal_pose_Xd_, nsteps_, arrival_time_, des_trajectory_);
    if (traj_is_valid_) {
        multi_traj_vec_.clear(); //whenever plan a single traj, make this default start of multi-seg trajectory
        multi_traj_vec_.push_back(des_trajectory_);
    }
    traj_plan_wrapup();
    return traj_is_valid_;
}

//handy fnc to get current  tool pose
// gets current joint angles, does fwd kin, includes tool xform
// converts result to a geometry_msgs::PoseStamped
//fills in member var current_gripper_stamped_pose_

void ArmMotionInterface::compute_tool_stamped_pose(void) {
    //get_joint_angles(); //will update q_vec
    q_vec_ = q_vec_arm_Xd_;
    affine_flange_wrt_base_ =
            pFwdSolver_->fwd_kin_solve(q_vec_); //rtns pose of flange w/rt base frame
    // ^tool_A_flange * ^flange_A_base
    //affine_tool_wrt_flange_
    affine_tool_wrt_base_ = affine_tool_wrt_flange_*affine_flange_wrt_base_;

    current_gripper_pose_ = xformUtils.transformEigenAffine3dToPose(affine_tool_wrt_base_);
    current_gripper_stamped_pose_.pose = current_gripper_pose_;
    current_gripper_stamped_pose_.header.stamp = ros::Time::now();
    current_gripper_stamped_pose_.header.frame_id = urdf_base_frame_name_;
}

Eigen::Affine3d ArmMotionInterface::xform_gripper_pose_to_affine_flange_wrt_base(geometry_msgs::PoseStamped des_pose_gripper) {
    Eigen::Affine3d affine_flange_wrt_base;
    tf::StampedTransform flange_stf, flange_wrt_base_stf;
    geometry_msgs::PoseStamped flange_gmps, flange_wrt_base_gmps;
    //convert des gripper pose to a stamped transform, so we can do more transforms
    //ROS_WARN("xform_gripper_pose_to_affine_flange_wrt_base: input pose-stamped: ");
    //xformUtils.printStampedPose(des_pose_gripper);
    tf::StampedTransform gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(des_pose_gripper, "generic_gripper_frame");
    //convert to transform of corresponding tool flange w/rt whatever reference frame_id
    //ROS_INFO("gripper_stf: ");
    //xformUtils.printStampedTf(gripper_stf);

    bool mult_ok = xformUtils.multiply_stamped_tfs(gripper_stf, generic_toolflange_frame_wrt_gripper_frame_stf_, flange_stf);
    //ROS_INFO("flange_stf");
    //xformUtils.printStampedTf(flange_stf); 
    if (!mult_ok) {
        ROS_WARN("stf multiply not legal! ");
    } //should not happen
    //ROS_INFO("corresponding flange frame: ");
    //xformUtils.printStampedTf(flange_stf);
    //convert stamped ‘tf::StampedTransform to geometry_msgs::PoseStamped
    flange_gmps = xformUtils.get_pose_from_stamped_tf(flange_stf);
    //change the reference frame from whatever to "base":
    bool tferr = true;
    int ntries = 0;
    ros::Time now = ros::Time::now();
    while (tferr) {
        tferr = false;
        try {
            tfListener_->waitForTransform("base_link", flange_gmps.header.frame_id,
                    now, ros::Duration(1.0)); //waits up to N sec for transform to be valid
            tfListener_->transformPose("base_link", flange_gmps, flange_wrt_base_gmps);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; transform problem; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }

    //ROS_INFO("corresponding flange frame w/rt base frame: ");
    //xformUtils.printStampedPose(flange_wrt_base_gmps);  
    //convert this to an affine.  parent and child frame id's are lost, so we'll have to remember what this means
    affine_flange_wrt_base = xformUtils.transformPoseToEigenAffine3d(flange_wrt_base_gmps);
    //ROS_WARN("xform_gripper_pose_to_affine_flange_wrt_base returning affine: ");
    display_affine(affine_flange_wrt_base);
    //xformUtils.printAffine(affine_flange_wrt_base);
    return affine_flange_wrt_base;
}

//handy utility, just to print data to screen for Affine objects

void ArmMotionInterface::display_affine(Eigen::Affine3d affine) {
    cout << "Affine origin: " << affine.translation().transpose() << endl;
    cout << "Affine rotation: " << endl;
    cout << affine.linear() << endl;
}

//repetive code, so make it a fnc;
//evals if traj is valid and stuffs result message

void ArmMotionInterface::traj_plan_wrapup() {

    if (traj_is_valid_) {
        computed_arrival_time_ = des_trajectory_.points.back().time_from_start.toSec();
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }
}



/*
//given q_start, compute the tool-flange pose, and compute a path to move delta_p with R fixed
// return the optimized joint-space path in optimal_path
// this fnc can be used fairly generally--e.g., special cases such as 20cm descent from current arm pose 
// this fnc is used within rt_arm_plan_path_current_to_goal_dp_xyz()

bool ArmMotionInterface::plan_cartesian_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p) {
    //ROS_INFO("attempting Cartesian path plan for delta-p = %f, %f, %f",delta_p_(0),delta_p_(1),delta_p_(2));
    cout << delta_p.transpose() << endl;
    // this fnc will put the optimal_path result in a global vector, accessible by main
    path_is_valid_ = pCartTrajPlanner_->cartesian_path_planner_delta_p(q_start, delta_p, optimal_path_);
    if (path_is_valid_) {
        ROS_INFO("plan_cartesian_delta_p: computed valid delta-p path");
        q_vec_start_resp_ = optimal_path_.front();
        q_vec_end_resp_ = optimal_path_.back();
    } else {
        ROS_WARN("plan_cartesian_delta_p: path plan attempt not successful");
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
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
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
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::SUCCESS;
        cart_result_.computed_arrival_time = computed_arrival_time_;
        cart_move_as_.setSucceeded(cart_result_);
    } else {
        cart_result_.return_code = arm_motion_action::arm_interfaceResult::PATH_NOT_VALID;
        cart_result_.computed_arrival_time = -1.0; //impossible arrival time        
        cart_move_as_.setSucceeded(cart_result_); //the communication was a success, but not the computation 
    }

    return path_is_valid_;
}
 */


