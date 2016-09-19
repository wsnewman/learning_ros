//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
#include <object_grabber/object_grabber.h>
using namespace std;

ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle) : nh_(*nodehandle),
object_grabber_as_(nh_, "objectGrabberActionServer", boost::bind(&ObjectGrabber::executeCB, this, _1), false),
armMotionCommander(nodehandle),
baxterGripper(nodehandle)
{
    ROS_INFO("in constructor of ObjectGrabber");
    gripper_table_z_ = 0.05; //gripper origin height above torso for grasp of cyl on table
    L_approach_ = 0.25; //distance to slide towards cylinder
    z_depart_ = 0.2; //height to lift cylinder

    dz_approach_offset_ = 0.1; //for vertical approach, approach from this far away relative to object top surface
    //define a gripper orientation for power-grasp approach of upright cylinder
    gripper_n_des_ << 0, 0, 1; //gripper x-axis points straight up;
    gripper_theta_ = M_PI / 3.0; //approach yaw angle--try this, reaching out and to the left
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

    //get the tool transform from tf: this is specialized for Baxter right_gripper w/rt right_hand (flange frame)
    a_right_gripper_frame_wrt_flange = get_right_tool_transform();

    object_grabber_as_.start(); //start the server running
}

// function to grasp an object (e.g. toy_block) from above;
// provide object pose assuming pose origin is on top surface of object 
// assume object frame z-axis is "up" (normal to top face, pointing outwards)
// and object-frame x-axis is along the object's major axis 
// grasp pose for this fnc will have fingers below top surface of block,  e.g. delta-z = 0.03 should work for toy_block,
// and will assume fingers straddle the object along the minor axis
// start from safe distance above (approach pose); descend to grasp pose (carefully); depart back to approach pose

// grasp_z_offset: starting from gripper tip flush with top surface of object, gripper should descend this much before grasping object
//  this value depends on the specific object and should be encoded with object-type description

//note--block_pose_wrt_torso is already assured to be w/rt torso, due to vetting by executeCB()

//fnc to make a Cartesian move to specified flange pose:
int ObjectGrabber::move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    std::vector<Vectorq7x1> q_solns;
    //plan move from current pose to approach pose:
    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);
    
    //is plan successful?
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
      ROS_WARN("cannot move to specified approach pose");
      return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_POSE_CARTESIAN_MOVE;
    }

    //if here, plan is good, so send command to execute planned motion
    ROS_INFO("sending command to execute planned path to approach pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS; 
}

//pure, careful, precise motion from current pose to precise goal...e.g. vertical approach
int ObjectGrabber::fine_move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {  
    int planner_rtn_code,execute_return_code;
    ROS_INFO("planning hi-res move");
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
      ROS_WARN("desired fine motion is not feasible");
      return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_POSE_CARTESIAN_MOVE;
    }
    //if succeed to here, send command to execute planned motion
        ROS_INFO("sending command to execute planned hi-res path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS; 
}

//open-gripper func confirms opening before returning--or times out and complains
//"open" is defined by the filtered finger opening exceeding a provided threshold 
int ObjectGrabber::open_gripper(double open_val_test) {
    //open the gripper:
    ROS_INFO("opening gripper");
    double dt = 0.01;
    double stopwatch = 0.0;

    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>%f", baxterGripper.get_right_gripper_pos(), open_val_test);
    while ((baxterGripper.get_right_gripper_pos() < open_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        stopwatch += dt;
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(dt).sleep();
    }
    if (baxterGripper.get_right_gripper_pos() > open_val_test) {
        ROS_INFO("gripper is open to > %f", open_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_OPEN;
    } else {
        ROS_WARN("timeout expired without opening gripper");
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}

//close-gripper waits for finger separation to be less than test value,
// else times out
int ObjectGrabber::close_gripper(double close_val_test) {
    ROS_INFO("closing gripper");
    double dt = 0.01;
    double stopwatch = 0.0;
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    //g_right_gripper_pos=110;
    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    while ((baxterGripper.get_right_gripper_pos() > close_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        stopwatch += dt;
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(dt).sleep();
    }

    if (baxterGripper.get_right_gripper_pos() < close_val_test) {
        ROS_INFO("gripper is closed to < %f", close_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_CLOSED;
    } else {
        ROS_WARN("timeout expired without closing gripper");
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}

//fnc uses tf to transform a provided pose into the torso frame
geometry_msgs::PoseStamped ObjectGrabber::convert_pose_to_torso_frame(geometry_msgs::PoseStamped pose_stamped) {
    //convert object pose into object pose w/rt torso:
    geometry_msgs::PoseStamped pose_stamped_wrt_torso;
    bool valid_tf = false;
    while (!valid_tf) {
        try {
            tfListener.transformPose("torso", pose_stamped, pose_stamped_wrt_torso);
            valid_tf = true;
        } catch (tf::TransformException &ex) {
            ROS_WARN("transform not valid...retrying");
            valid_tf = false;
            ros::Duration(0.1).sleep();
        }
    }
    ROS_INFO("desired pose_stamped_wrt_torso_ origin: %f, %f, %f", pose_stamped_wrt_torso.pose.position.x,
            pose_stamped_wrt_torso.pose.position.y, pose_stamped_wrt_torso.pose.position.z);
    return pose_stamped_wrt_torso;
}

            
            
//fnc to compute gripper pose from block pose, per a specific grasp strategy
Eigen::Affine3d ObjectGrabber::block_grasp_transform(Eigen::Affine3d block_affine) {
    Eigen::Affine3d gripper_affine;
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
    //gripper_affine is defined to have origin coincident w/ block-frame origin, but z-axis antiparallel to block z-axis
    // and x-axis parallel to block-frame x-axis
    gripper_affine.linear() = R_gripper; //populate affine w/ orientation
    gripper_affine.translation() = block_affine.translation(); //and origin
    return gripper_affine;  
}

//same as above, except uses stamped poses:
geometry_msgs::PoseStamped ObjectGrabber::block_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    Eigen::Affine3d block_affine, gripper_affine;
    geometry_msgs::PoseStamped gripper_pose;
    block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose); 
    gripper_affine =  block_grasp_transform(block_affine); // convert from block frame to gripper frame
    gripper_pose.header = block_pose.header;
    gripper_pose.pose = xformUtils.transformEigenAffine3dToPose(gripper_affine);
    return gripper_pose;
} 

//does two steps: convert from block pose to gripper pose, then gripper pose to flange pose
// could use TF for 2nd step
geometry_msgs::PoseStamped ObjectGrabber::block_to_flange_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    geometry_msgs::PoseStamped flange_pose_for_block_grasp;
    Eigen::Affine3d block_affine, gripper_affine, flange_affine;
    block_affine = block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose); 
    gripper_affine =  block_grasp_transform(block_affine); // convert from block frame to gripper frame
    flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange.inverse(); // from gripper frame to flange frame
    flange_pose_for_block_grasp.header = block_pose.header; //and from affine to stamped pose
    flange_pose_for_block_grasp.pose = xformUtils.transformEigenAffine3dToPose(flange_affine);
    return flange_pose_for_block_grasp;
}
int ObjectGrabber::grasp_from_above(geometry_msgs::PoseStamped des_flange_grasp_pose, double approach_dist) {
    int rtn_val;
    //geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    geometry_msgs::PoseStamped des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_grasp_affine, flange_depart_affine;
    //convert right-gripper pose to an affine--SHOULD MAKE SURE POSE IS WRT TORSO!!
    flange_grasp_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_grasp_pose.pose);

    //the following coords are all right-arm toolflange frame w/rt torso
    Eigen::Vector3d approach_origin, flange_grasp_origin;
    flange_grasp_origin = flange_grasp_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;
    R = flange_grasp_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;

    a_flange_grasp_ = flange_grasp_affine; //copy to mem var
    // compute  a_flange_approach_, a_flange_depart_,
    a_flange_approach_ = a_flange_grasp_;
    //approach from distance dz_approach_offset_    
    a_flange_approach_.translation() = a_flange_grasp_.translation() - toolflange_z_axis*approach_dist; 
    a_flange_depart_ = a_flange_approach_; //choose to depart to same pose as approach 
    ROS_INFO(" flange pose for object grasp: ");
    cout << "origin: " << a_flange_grasp_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_grasp_.linear() << endl;

    ROS_INFO("flange pose for approach: ");
    cout << "origin: " << a_flange_approach_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_approach_.linear() << endl;

    //open the gripper:
    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE==gripper_status) {
        return gripper_status;//failure to open gripper; return diagnostic
    }

       
    int move_to_rtn_code;
    //plan/execute Cartesian move to approach pose
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(a_flange_approach_);
    move_to_rtn_code = move_flange_to(des_flange_approach_pose); 
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code; // give up--and send diagnostic code
    }

    // make a careful approach to grasp pose, along gripper-z direction (e.g., vertical)
    //plan path to grasp pose:
    move_to_rtn_code = fine_move_flange_to(des_flange_grasp_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code; // give up--and send diagnostic code
    }    
 
    //close the gripper, hopefully to grasp the part
    gripper_status = close_gripper(90.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE==gripper_status) {
        return gripper_status;//failure to open gripper; return diagnostic
    }
 
    ros::Duration(1).sleep(); //some extra settling time for grasp
    if (baxterGripper.get_right_gripper_pos() < object_grabber::object_grabberGoal::TOY_BLOCK_FINGER_OPENING - 10) {
        return object_grabber::object_grabberResult::FAILED_OBJECT_NOT_IN_GRIPPER;
    }
    //depart vertically:
    des_flange_depart_pose = des_flange_approach_pose;
   move_to_rtn_code = move_flange_to(des_flange_depart_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code; // give up--and send diagnostic code
    }    
   

    return object_grabber::object_grabberResult::SUCCESS;
}


//drop-off fnc;
// specify gripper pose for drop-off, so this can be used with objects of different heights and different grasp transforms;
// the parent pgm must compute the gripper pose that corresponds to desired object pose, taking into account grasp transform and
// object dimensions

int ObjectGrabber::dropoff_from_above(geometry_msgs::PoseStamped des_flange_dropoff_pose, double approach_dist)  {
        int move_to_rtn_code;
        geometry_msgs::PoseStamped des_flange_approach_pose;
        Eigen::Affine3d dropoff_flange_affine, approach_flange_affine;
        //next two steps already handled by fnc block_to_flange_grasp_transform()
        //dropoff_gripper_affine = xformUtils.transformPoseToEigenAffine3d(dropoff_gripper_pose_wrt_torso.pose);
        //dropoff_flange_affine = dropoff_gripper_affine * a_right_gripper_frame_wrt_flange.inverse();        
        dropoff_flange_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_dropoff_pose.pose);


    //the following coords are all right-arm toolflange frame w/rt torso
    Eigen::Vector3d approach_flange_origin, dropoff_flange_origin;
    dropoff_flange_origin = dropoff_flange_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;

    R = dropoff_flange_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;
    cout << "offset by " << approach_dist << " to descend to grasp pose" << endl;
    
    approach_flange_affine = dropoff_flange_affine; //derive flange pose for approach; start from drop-off pose
    //and offset in z-axis direction by -approach_dist
    approach_flange_affine.translation() = dropoff_flange_affine.translation() - toolflange_z_axis*approach_dist; 
    
    //convert these back to stamped poses:
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(approach_flange_affine);
    des_flange_dropoff_pose.header.frame_id= "torso";
    des_flange_dropoff_pose.pose = xformUtils.transformEigenAffine3dToPose(dropoff_flange_affine);    
    ROS_INFO("attempting move to approach pose");
    move_to_rtn_code = move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d",move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }
    //now do a careful approach move along gripper-z axis:
    ROS_INFO("attempting fine-move approach to drop-off");
    move_to_rtn_code = fine_move_flange_to(des_flange_dropoff_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d",move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }     
    
    //open the gripper:
    ROS_INFO("releasing the part");
    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE==gripper_status) {
        return gripper_status;//failure to open gripper; return diagnostic
    }    
    
    //depart, carefully:
    ROS_INFO("computing/executing depart move");
    move_to_rtn_code = fine_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d",move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }       

    return object_grabber::object_grabberResult::SUCCESS;
}

//this fnc is intended to grasp a cylinder that is oriented vertically;
//approach w/ tool-flange z-axis parallel to table surface
// and with vector between fingertips parallel to table surface
// for Baxter gripper, this means orient right_gripper x-axis "up"
// thus can slide horizontally along tool-flange z-axis to approach the cylinder from the side

// assumes object origin is in the center of the cylinder, and z-axis is through cylinder axis of symmetry
// assumes fingers should close at z-height of object origin w/ fingertips diametrically opposed
// if cylinder is too short to allow for clearance, need to specify an additional vertical offset

// After grasping, depart normal to the table surface, e.g. as in picking up a bottle containing fluid
// NEEDS REVIEW: all frames for motion planning should be expressed w/respect to tool flange frame

//note--object_pose is already known to be w/rt torso, due to executeCB() vetting

int ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    geometry_msgs::PoseStamped des_flange_grasp_pose, des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_approach_affine, flange_grasp_affine, flange_depart_affine;

    int rtn_val, execute_return_code;
    //send a command to plan a joint-space move to pre-defined pose:   
    rtn_val = armMotionCommander.plan_move_to_pre_pose();
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path to pre-pose:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion to pre-pose is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }

    //inquire re/ right-arm joint angles:
    rtn_val = armMotionCommander.rt_arm_request_q_data();

    Eigen::Affine3d object_affine;
    object_affine =
            xformUtils.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    //grasp_origin_(2) = gripper_table_z_; //except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;
    // convert to flange pose:
    flange_grasp_affine = a_gripper_grasp_ * a_right_gripper_frame_wrt_flange.inverse();

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    flange_approach_affine = flange_grasp_affine;
    flange_approach_affine.translation() = flange_grasp_affine.translation() - gripper_b_des_*L_approach_;

    // after have cylinder grasped, move purely upwards by z_depart:
    flange_depart_affine = flange_grasp_affine;
    flange_depart_affine.translation() = flange_grasp_affine.translation() + gripper_n_des_*z_depart_;

    //open the gripper:
    ROS_INFO("opening gripper");
    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() < 95.0) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }

    //start w/ a jnt-space move from current pose to approach pose:
    int planner_rtn_code;
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_approach_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_approach_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }

    //slide to can:
    ROS_INFO("planning hi-res approach to grasp pose");
    des_flange_grasp_pose.header.frame_id = "torso";
    des_flange_grasp_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_grasp_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_grasp_pose);
    //optionally, rescale this path to slow it down:
    double time_stretch_factor = 3.0; // tune this approach speed; e.g., slow down by factor of 3
    planner_rtn_code = armMotionCommander.rt_arm_timestretch_planned_path(time_stretch_factor);

    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }


    //close the gripper:
    ROS_INFO("closing gripper");
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    //g_right_gripper_pos=110;
    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() > 90.0) {
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }
    ros::Duration(1).sleep(); //some extra settling time for grasp

    //depart vertically:
    des_flange_depart_pose.header.frame_id = "torso";
    des_flange_depart_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_depart_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_depart_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }
    return object_grabber::object_grabberResult::SUCCESS;

}


//callback: at present, hard-coded for two specific objects;
//extend this to add more grasp strategies for more objects

void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {

    int object_code = goal->object_code;
    //object_pose_stamped_ = goal->object_frame;

    int object_grabber_rtn_code;
    switch (object_code) {
        case object_grabber::object_grabberGoal::GRAB_UPRIGHT_CYLINDER:
            //case object_grabber::object_grabberGoal::UPRIGHT_CYLINDER:
            ROS_INFO("case GRAB_UPRIGHT_CYLINDER");
            object_pose_stamped_ = goal->desired_frame;
            object_pose_stamped_wrt_torso_= convert_pose_to_torso_frame(object_pose_stamped_);

            object_grabber_rtn_code = vertical_cylinder_power_grasp(object_pose_stamped_wrt_torso_);
            grab_result_.return_code = object_grabber_rtn_code;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabberGoal::GRAB_TOY_BLOCK:
            //case object_grabber::object_grabberGoal::TOY_BLOCK:
            ROS_INFO("case GRAB_TOY_BLOCK");
            object_pose_stamped_ = goal->desired_frame;
            object_pose_stamped_wrt_torso_= convert_pose_to_torso_frame(object_pose_stamped_);
            des_flange_pose_stamped_wrt_torso_ = block_to_flange_grasp_transform(object_pose_stamped_wrt_torso_);
            
            //make this more general--convert block pose to gripper pose via grasp transform
            //then can simplify grasp_from_above fnc
            //object_grabber_rtn_code = grasp_from_above(object_pose_stamped_wrt_torso_,
            object_grabber_rtn_code = grasp_from_above(des_flange_pose_stamped_wrt_torso_,
                    object_grabber::object_grabberGoal::TOY_BLOCK_APPROACH_DIST);

            grab_result_.return_code = object_grabber_rtn_code;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        case object_grabber::object_grabberGoal::PLACE_TOY_BLOCK:
            ROS_INFO("case PLACE_TOY_BLOCK");
            object_pose_stamped_ = goal->desired_frame; //make sure is expressed w/rt torso frame
            object_pose_stamped_wrt_torso_= convert_pose_to_torso_frame(object_pose_stamped_);            
            //need to consider grasp transform, from object frame to flange frame
            des_flange_pose_stamped_wrt_torso_ = block_to_flange_grasp_transform(object_pose_stamped_wrt_torso_);

            object_grabber_rtn_code = dropoff_from_above(des_flange_pose_stamped_wrt_torso_,
                    object_grabber::object_grabberGoal::TOY_BLOCK_APPROACH_DIST);
            grab_result_.return_code = object_grabber_rtn_code;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        //partial move commands--lower level than above; interpret object_pose_stamped_wrt_torso_ as desired FLANGE pose
        case object_grabber::object_grabberGoal::MOVE_FLANGE_TO:
           ROS_INFO("case MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_= convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = move_flange_to(des_flange_pose_stamped_wrt_torso_);
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        case object_grabber::object_grabberGoal::FINE_MOVE_FLANGE_TO:
           ROS_INFO("case FINE_MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_= convert_pose_to_torso_frame(des_flange_pose_stamped_);           
            grab_result_.return_code = fine_move_flange_to(des_flange_pose_stamped_wrt_torso_);
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        case object_grabber::object_grabberGoal::OPEN_GRIPPER:
            open_gripper(goal->gripper_test_val);
            break;
        case object_grabber::object_grabberGoal::CLOSE_GRIPPER:
            close_gripper(goal->gripper_test_val);
            break;            
        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }

}

Eigen::Affine3d ObjectGrabber::get_right_tool_transform(void) {
    //get transform from right_gripper to right_hand

    ROS_INFO("listening for gripper-to-toolflange transform:");
    tf::StampedTransform stf_gripper_wrt_flange;
    bool tferr = true;
    ROS_INFO("waiting for tf between right gripper and right tool flange...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("right_hand", "right_gripper", ros::Time(0), stf_gripper_wrt_flange);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("right gripper to right tool flange tf is good");
    xformUtils.printStampedTf(stf_gripper_wrt_flange);
    tf::Transform tf_kinect_wrt_base = xformUtils.get_tf_from_stamped_tf(stf_gripper_wrt_flange);
    Eigen::Affine3d affine_gripper_wrt_flange = xformUtils.transformTFToAffine3d(tf_kinect_wrt_base);

    std::cout << "affine rotation: " << std::endl;
    std::cout << affine_gripper_wrt_flange.linear() << std::endl;
    std::cout << "affine offset: " << affine_gripper_wrt_flange.translation().transpose() << std::endl;
    return affine_gripper_wrt_flange;
}
