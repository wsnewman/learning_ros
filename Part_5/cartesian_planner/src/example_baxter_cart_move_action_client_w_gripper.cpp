// example_baxter_cart_move_action_client_w_gripper: 
// wsn, August, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"
// as well as simple_baxter_gripper_interface

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <cartesian_planner/baxter_arm_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include<simple_baxter_gripper_interface/simple_baxter_gripper_interface.h>
using namespace std;

class InitVars {
    public:
    InitVars();
    XformUtils xformUtils;
    //double right_gripper_pos_;
    Eigen::Affine3d get_right_tool_transform(void);
    geometry_msgs::PoseStamped perceived_object_poseStamped,desired_toolflange_poseStamped,approach_toolflange_poseStamped;
    Eigen::Affine3d flange_approach_affine,gripper_approach_affine,object_affine;
    baxter_core_msgs::EndEffectorCommand gripper_cmd_open, gripper_cmd_close; 
    Eigen::Affine3d affine_gripper_wrt_flange;
    Eigen::Affine3d affine_des_flange;
};
InitVars::InitVars() {
    //right_gripper_pos_= -1;
    //hard code an object pose; later, this will come from perception
    perceived_object_poseStamped.header.frame_id = "torso";//define a hard-coded desired gripper pose
    perceived_object_poseStamped.header.stamp = ros::Time::now();    
    perceived_object_poseStamped.pose.position.x = 0.5;
    perceived_object_poseStamped.pose.position.y = -0.35;
    perceived_object_poseStamped.pose.position.z = -0.145; //-0.112--> gripper frame origin at top of block
    
    perceived_object_poseStamped.pose.orientation.x = 0;
    perceived_object_poseStamped.pose.orientation.y = 0;
    perceived_object_poseStamped.pose.orientation.z = 0.842;
    perceived_object_poseStamped.pose.orientation.w = 0.54;

    perceived_object_poseStamped.header.frame_id = "torso";  

    
    //convert right-gripper pose to an affine--SHOULD MAKE SURE POSE IS WRT TORSO!!
    object_affine = xformUtils.transformPoseToEigenAffine3d(perceived_object_poseStamped.pose);
    //derive gripper approach pose from block pose:
   //compute a gripper pose with z-axis anti-parallel to object z-axis,
    // and x-axis coincident with object x-axis
    Eigen::Vector3d x_axis, y_axis, z_axis;
    Eigen::Matrix3d R_object, R_gripper;
    R_object = object_affine.linear(); //get 3x3 matrix from affine
    x_axis = R_object.col(0); //extract the x axis
    z_axis = -R_object.col(2); //define gripper z axis antiparallel to object z-axis
    y_axis = z_axis.cross(x_axis); // construct a right-hand coordinate frame
    R_gripper.col(0) = x_axis; //populate orientation matrix from axis directions
    R_gripper.col(1) = y_axis;
    R_gripper.col(2) = z_axis;
    gripper_approach_affine.linear() = R_gripper; //populate affine w/ orientation
    gripper_approach_affine.translation() = object_affine.translation(); //and origin
    cout << "gripper_approach_affine origin: " << gripper_approach_affine.translation().transpose() << endl;
    cout << "gripper_approach_affine R matrix: " << endl;
    cout << gripper_approach_affine.linear() << endl;

    affine_gripper_wrt_flange = get_right_tool_transform();

    affine_des_flange = gripper_approach_affine*affine_gripper_wrt_flange.inverse();

    desired_toolflange_poseStamped.header = perceived_object_poseStamped.header;
    desired_toolflange_poseStamped.pose = xformUtils.transformEigenAffine3dToPose(affine_des_flange);
    approach_toolflange_poseStamped = desired_toolflange_poseStamped;
    approach_toolflange_poseStamped.pose.position.z+= 0.1; //elevate gripper for pre-approach
    ROS_INFO("desired toolflange pose: ");
    xformUtils.printStampedPose(desired_toolflange_poseStamped);   
    
        
    gripper_cmd_open.command ="go";
    gripper_cmd_open.args = "{'position': 100.0}'";
    gripper_cmd_open.sender = "gripper_publisher";
    gripper_cmd_close.command ="go";
    gripper_cmd_close.args = "{'position': 0.0}'";
    gripper_cmd_close.sender = "gripper_publisher"; 
}

Eigen::Affine3d InitVars::get_right_tool_transform(void) {
   //get transform from right_gripper to right_hand
    tf::TransformListener tfListener;
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



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_move_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    
    //instantiate a Baxter arm-motion commander object to interface w/ cartMoveActionServer
    ArmMotionCommander arm_motion_commander(&nh);
    InitVars initVars;
    BaxterGripper baxterGripper(&nh); 
    
    while(baxterGripper.get_right_gripper_pos()<-0.5) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        ROS_INFO("waiting for gripper position; pos = %f",baxterGripper.get_right_gripper_pos());        
    }
        
    int rtn_val;
    
    ROS_INFO("closing gripper");  
    baxterGripper.right_gripper_close();
    ros::Duration(1.0).sleep();
    ROS_INFO("opening gripper");
    baxterGripper.right_gripper_open(); 
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>95",baxterGripper.get_right_gripper_pos());    
    while(baxterGripper.get_right_gripper_pos()<95.0) {
            baxterGripper.right_gripper_open();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f",baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }        
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("planning move to pre-pose:");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //plan a path from current pose to specified, desired flange pose
    ROS_INFO("planning path");   
        rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_flange_pose(initVars.approach_toolflange_poseStamped);
               //send command to execute planned motion
           ROS_INFO("sending command to execute planned path to approach pose:");
           rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

     //plan path to grasp pose:
        ROS_INFO("planning hi-res descent to grasp pose");
    rtn_val=arm_motion_commander.rt_arm_plan_fine_path_current_to_goal_flange_pose(initVars.desired_toolflange_poseStamped);
    
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS)  { 
       //optionally, rescale this path to slow it down:
        //double time_stretch_factor = 3.0; // tune this approach speed; e.g., slow down by factor of 3
        //rtn_val=arm_motion_commander.rt_arm_timestretch_planned_path(time_stretch_factor);
        

            //send command to execute planned motion
           ROS_INFO("sending command to execute planned path:");
           rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    }
    else {
        ROS_WARN("desired motion is not feasible");
    }
    ROS_INFO("closing gripper");   
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    //g_right_gripper_pos=110;
    ROS_INFO("gripper pos = %f",baxterGripper.get_right_gripper_pos());    
    while(baxterGripper.get_right_gripper_pos()>90.0) {
            baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f",baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }
    ros::Duration(1).sleep(); //some extra settling time for grasp
    
    ROS_INFO("planning motion to depart pose");
    rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_flange_pose(initVars.approach_toolflange_poseStamped);
    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path to depart pose:");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    return 0;
}

