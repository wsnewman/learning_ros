// example_ur10_cart_move_action_client: 
// wsn, Nov, 2016
// illustrates use of ur10_cart_move_as, action server called "cartMoveActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/ur10_cart_moveAction.h>
#include <cartesian_planner/ur10_arm_motion_commander.h>
#include <ur_fk_ik/ur_kin.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_move_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    
    //instantiate a UR10 arm-motion commander object to interface w/ cartMoveActionServer
    ArmMotionCommander arm_motion_commander(&nh);
    
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int ans;
    geometry_msgs::PoseStamped tool_pose, flange_pose, pre_pose_flange_pose;
    
    //illustrate various functions of arm motion commander:
    ROS_INFO("sending a test goal");
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("planning move to pre-pose:");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path");
    rtn_val=arm_motion_commander.execute_planned_path();
    
    //inquire re/  joint angles:
    ROS_INFO("requesting joint angles");
    rtn_val=arm_motion_commander.request_q_data();
    
    //inquire re/ flange pose w/rt base:    
    ROS_INFO("requesting current  flange pose w/rt base");
    rtn_val=arm_motion_commander.request_flange_pose_wrt_base();    
    
    //do a joint-space move; get the start angles:
    joint_angles = arm_motion_commander.get_joint_angles();
    
    //increment all of the joint angles by a fixed amt:
    ROS_INFO("defining a joint-space move: increment all jnt angles by 0.2 rad");
    for (int i=0;i<NJNTS;i++) joint_angles[i]+=0.2;
    
    //try planning a joint-space motion to this new joint-space pose:
    ROS_INFO("planning jspace path to this pose");
    rtn_val=arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);
    //send command to execute planned motion
    cout<<"enter 1 to execute plan: ";
    cin>>ans;    
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.request_q_data();
    
    //return to pre-defined pose:
    ROS_INFO("requesting path plan back to pre-pose");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose(); 
    cout<<"enter 1 to execute plan: ";
    cin>>ans;     
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.execute_planned_path();    

    //get flange pose
    rtn_val = arm_motion_commander.request_flange_pose_wrt_base();
    flange_pose = arm_motion_commander.get_flange_pose_stamped();
    pre_pose_flange_pose = flange_pose; //remember this pose
    ROS_INFO("flange_pose origin: %f, %f, %f",flange_pose.pose.position.x,flange_pose.pose.position.y,flange_pose.pose.position.z);

    //tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning cartesian path to move  dp_x = 0.2, dp_y = 0.5");
    
    flange_pose.pose.position.y += 0.5; // move 50cm, along y in torso frame
    flange_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    ROS_INFO("desired pose origin: %f, %f, %f",flange_pose.pose.position.x,flange_pose.pose.position.y,flange_pose.pose.position.z);

    rtn_val=arm_motion_commander.plan_path_current_to_goal_flange_pose(flange_pose);
    //send command to execute planned motion
    cout<<"enter 1 to execute plan: ";
    cin>>ans;  
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.execute_planned_path();

    //try vector cartesian displacement at fixed orientation:
    std::cout<<"enter desired delta-z motion: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    ROS_INFO("planning path");
    rtn_val = arm_motion_commander.plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::ur10_cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           ROS_INFO("sending command to execute planned path:");
           rtn_val=arm_motion_commander.execute_planned_path();
    }
    else {
        ROS_WARN("desired motion is not feasible");
    }
    //plan a joint-space path back to flange_pose0
    ROS_INFO("planning a joint-space path back to pre-pose flange pose...");
    //plan_jspace_path_current_to_flange_pose(geometry_msgs::PoseStamped des_pose)
    rtn_val=arm_motion_commander.plan_jspace_path_current_to_flange_pose(pre_pose_flange_pose);
    
    double time_stretch_factor;
    cout<<"enter time scale factor for this move: ";
    cin>>time_stretch_factor;
 
    rtn_val= arm_motion_commander.timestretch_planned_path( time_stretch_factor);

    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.execute_planned_path();
    
    return 0;
}

