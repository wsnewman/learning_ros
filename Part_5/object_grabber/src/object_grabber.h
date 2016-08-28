//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ObjectGrabber {
private:
    ros::NodeHandle nh_;
    XformUtils xformUtils;
    //messages to send/receive cartesian goals / results:
    object_grabber::object_grabberGoal grab_goal_;
    object_grabber::object_grabberResult grab_result_; 
    object_grabber::object_grabberFeedback grab_fdbk_;    
    geometry_msgs::PoseStamped object_pose_stamped_;
    int object_code_;
    std_msgs::Bool gripper_open,gripper_close;
    
    double gripper_theta_;
    double z_depart_,L_approach_;
    double gripper_table_z_;
    Eigen::Vector3d gripper_b_des_;
    Eigen::Vector3d gripper_n_des_;
    Eigen::Vector3d gripper_t_des_;
    Eigen::Vector3d grasp_origin_,approach_origin_,depart_origin_;
    Eigen::Matrix3d R_gripper_vert_cyl_grasp_;
    Eigen::Affine3d a_gripper_start_,a_gripper_end_;
    Eigen::Affine3d a_gripper_approach_,a_gripper_depart_, a_gripper_grasp_;
    Eigen::Affine3d a_right_gripper_frame_wrt_flange;
    Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    ros::Publisher gripper_publisher;
    
    actionlib::SimpleActionServer<object_grabber::object_grabberAction> object_grabber_as_;
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal);  
    
    void vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose);    
    void grasp_from_approach_pose(geometry_msgs::PoseStamped approach_pose, double approach_dist);

public:

        ObjectGrabber(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
        void set_right_tool_xform(Eigen::Affine3d xform) { a_right_gripper_frame_wrt_flange = xform; };

    ~ObjectGrabber(void) {
    }
    //define some member methods here

};


//name this server; 
ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle): nh_(*nodehandle),
   object_grabber_as_(nh_, "objectGrabberActionServer", boost::bind(&ObjectGrabber::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
        ROS_INFO("in constructor of ObjectGrabber");
    // do any other desired initializations here, as needed
    gripper_table_z_ = 0.05; //gripper origin height above torso for grasp of cyl on table
    L_approach_ = 0.25; //distance to slide towards cylinder
    z_depart_ = 0.2; //height to lift cylinder
    //define a gripper orientation for power-grasp approach of upright cylinder
    gripper_n_des_ << 0, 0, 1; //gripper x-axis points straight up;
    gripper_theta_ = M_PI/3.0; //approach yaw angle--try this, reaching out and to the left
    gripper_b_des_ << cos(gripper_theta_), sin(gripper_theta_), 0;
    gripper_t_des_ = gripper_b_des_.cross(gripper_n_des_);
    R_gripper_vert_cyl_grasp_.col(0) = gripper_n_des_;
    R_gripper_vert_cyl_grasp_.col(1) = gripper_t_des_;
    R_gripper_vert_cyl_grasp_.col(2) = gripper_b_des_;
    a_gripper_start_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_end_.linear() = R_gripper_vert_cyl_grasp_;   
    //define approach, grasp and depart poses: 
    a_gripper_approach_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_depart_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_grasp_.linear() = R_gripper_vert_cyl_grasp_;
    
    gripper_open.data= false;
    gripper_close.data=true;
    gripper_publisher = nh_.advertise<std_msgs::Bool>("gripper_open_close",1,true);

    object_grabber_as_.start(); //start the server running
}
//void ObjectGrabber::grasp_block_from_above(geometry_msgs::PoseStamped object_pose, double grasp_descent_dist) {
// assume object pose is z "up" and x along the major axis and origin is on top surface of object    
// descend by grasp_descent_dist, e.g. 0.04 should work for toy_block
// start from safe distance above, descend to approach pose, descend further to grasp pose, depart by same pre-approach dist
//}

// plan and execute a gripper and arm sequence to acquire an object
//for this function, provide pose of toy block, with frame defined to have
// origin on top face, z-axis normal to top face,
// and frame x-axis along major axis of block
// Assumes should open gripper, then toolflange should pre-position to block_pose,
// then move along toolflange z-axis by distance approach_dist, then close gripper,
// then depart along negative toolflange z-axis by approach_dist
//this function should be more general--should check frame_id of block_pose and transform it to torso frame, if necessary
// FIX THIS
void ObjectGrabber::grasp_from_approach_pose(geometry_msgs::PoseStamped block_pose, double approach_dist) {
   int rtn_val;
   geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    
    //inquire re/ right-arm joint angles:
    rtn_val=g_arm_motion_commander_ptr->rt_arm_request_q_data();
    
    Eigen::Affine3d flange_approach_affine,gripper_approach_affine,block_affine;
    //convert right-gripper pose to an affine--SHOULD MAKE SURE POSE IS WRT TORSO!!
    block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose);
    //derive gripper approach pose from block pose:
   //compute a gripper pose with z-axis anti-parallel to object z-axis,
    // and x-axis coincident with object x-axis
    Eigen::Vector3d x_axis, y_axis, z_axis;
    Eigen::Matrix3d R_object, R_gripper;
    R_object = block_affine.linear(); //get 3x3 matrix from affine
    x_axis = R_object.col(0); //extract the x axis
    z_axis = -R_object.col(2); //define gripper z axis antiparallel to object z-axis
    y_axis = z_axis.cross(x_axis); // construct a right-hand coordinate frame
    R_gripper.col(0) = x_axis; //populate orientation matrix from axis directions
    R_gripper.col(1) = y_axis;
    R_gripper.col(2) = z_axis;
    gripper_approach_affine.linear() = R_gripper; //populate affine w/ orientation
    gripper_approach_affine.translation() = block_affine.translation(); //and origin
    cout << "gripper_approach_affine origin: " << gripper_approach_affine.translation().transpose() << endl;
    cout << "gripper_approach_affine R matrix: " << endl;
    cout << gripper_approach_affine.linear() << endl;
    
    //convert this to corresponding tool-flange approach pose:
    // have position and orientation of gripper frame wrt flange frame, A_gripper/flange
    // s.t. x_flange = A_gripper/flange*x_gripper

    flange_approach_affine = gripper_approach_affine*a_right_gripper_frame_wrt_flange.inverse();
    ROS_INFO("equiv flange pose for gripper flush w/ object frame: ");
    cout << "flange_approach_affine origin: " << flange_approach_affine.translation().transpose() << endl;
    cout << "flange_approach_affine R matrix: " << endl;
    cout << flange_approach_affine.linear() << endl;
    
    //the following coords are all right-arm toolflange frame w/rt torso
    Eigen::Vector3d approach_origin;
    approach_origin = flange_approach_affine.translation();  
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;
    R = flange_approach_affine.linear();
    toolflange_z_axis = R.col(2);
    grasp_origin_ = approach_origin + toolflange_z_axis*approach_dist;
    a_gripper_grasp_.linear() = R;
    a_gripper_grasp_.translation() = grasp_origin_;
    ROS_INFO(" flange pose for object grasp: ");
    cout << "origin: " << a_gripper_grasp_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_gripper_grasp_.linear() << endl;
 
    a_gripper_approach_= a_gripper_grasp_; //start from here...
    approach_origin_ = grasp_origin_ - toolflange_z_axis*approach_dist*2.0; //back up this far
    a_gripper_approach_.translation() = approach_origin_;   
    ROS_INFO("flange pose for pre-approach: ");
     cout << "origin: " << a_gripper_approach_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_gripper_approach_.linear() << endl;  
     std::vector<Vectorq7x1> q_solns;   
    int nsolns = baxter_IK_solver_.ik_solve_approx(a_gripper_approach_,q_solns);
    ROS_INFO("flange approach pose nsolns = %d",nsolns);
    //int Baxter_IK_solver::ik_solve_approx(Eigen::Affine3d const& desired_flange_pose,std::vector<Vectorq7x1> &q_solns) // given desired hand pose, find all viable IK solns

    
    // depart along negative tool-flange z-axis
    depart_origin_ = approach_origin_;
    a_gripper_depart_.translation() = depart_origin_;    
    a_gripper_depart_.linear() = R;
    
    //open the gripper:
    gripper_publisher.publish(gripper_open);    
    
    //start w/ a jnt-space move from current pose to approach pose:
    int planner_rtn_code;
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_approach_);
     
    
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);
    
    //try to move here:
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
    
    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_grasp_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
    
    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();
    
    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_depart_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();   
}

void ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
   geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
   //make sure the object pose is in the torso frame; transform if necessary;
   //skip this for now
   int rtn_val;
       //send a command to plan a joint-space move to pre-defined pose:   
   rtn_val = g_arm_motion_commander_ptr->plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=g_arm_motion_commander_ptr->rt_arm_request_q_data();
    
    Eigen::Affine3d object_affine;
    object_affine = 
     g_arm_motion_commander_ptr->transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    grasp_origin_(2) = gripper_table_z_;//except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;
    
    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    approach_origin_ = grasp_origin_ - gripper_b_des_*L_approach_; 
    a_gripper_approach_.translation() = approach_origin_;
    
    // after have cylinder grasped, move purely upwards by z_depart:
    depart_origin_ = grasp_origin_ + gripper_n_des_*z_depart_;
    a_gripper_depart_.translation() = depart_origin_;
    
    //open the gripper:
    gripper_publisher.publish(gripper_open);
    
    //start w/ a jnt-space move from current pose to approach pose:
    int planner_rtn_code;
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_approach_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);
    
    //try to move here:
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
    
    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_grasp_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
    
    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();
    
    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_depart_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
} 


//callback: at present, hard-coded for Coke-can object;
//extend this to add more grasp strategies for more objects
// also, this code does NO error checking (e.g., unreachable); needs to be fixed!
void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {
 
   int object_code = goal->object_code;
   geometry_msgs::PoseStamped object_pose = goal->object_frame;
   switch(object_code) {
   case object_grabber::object_grabberGoal::COKE_CAN: 
     vertical_cylinder_power_grasp(object_pose);
     grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED; 
     object_grabber_as_.setSucceeded(grab_result_);
     break;
   case object_grabber::object_grabberGoal::TOY_BLOCK: 
       ROS_INFO("case TOY_BLOCK; received pose: ");
       
     grasp_from_approach_pose(object_pose, 
             object_grabber::object_grabberGoal::TOY_BLOCK_APPROACH_DIST);

     grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED; 
     object_grabber_as_.setSucceeded(grab_result_);
     break;     
   default:
             ROS_WARN("this object ID is not implemented");
             grab_result_.return_code = object_grabber::object_grabberResult::FAILED_OBJECT_UNKNOWN; 
             object_grabber_as_.setAborted(grab_result_);
            }
     
   
    //grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED; 
  
    //object_grabber_as_.setAborted(grab_result_); 
    //object_grabber_as_.setSucceeded(grab_result_); 
}
