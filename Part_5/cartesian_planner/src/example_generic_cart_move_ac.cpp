// example_generic_cart_move_ac: 
// wsn, Nov, 2016
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/cart_moveAction.h>
#include <cartesian_planner/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    ArmMotionCommander arm_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    geometry_msgs::PoseStamped tool_pose;
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("commanding move to waiting pose");
    rtn_val=arm_motion_commander.plan_move_to_waiting_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    rtn_val=arm_motion_commander.request_tool_pose();
    
    //do a joint-space move; get the start angles:
    joint_angles = arm_motion_commander.get_joint_angles();
    njnts = joint_angles.size();
    //increment all of the joint angles by a fixed amt:
    for (int i=0;i<njnts;i++) joint_angles[i]+=0.2;
    ROS_INFO("joint-space move, all joints +0.2 rad");
    //try planning a joint-space motion to this new joint-space pose:
    rtn_val=arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);

    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.request_q_data();
    
    //return to pre-defined pose:
    ROS_INFO("back to waiting pose");
    rtn_val=arm_motion_commander.plan_move_to_waiting_pose();
    rtn_val=arm_motion_commander.execute_planned_path();    

    //get tool pose
    rtn_val = arm_motion_commander.request_tool_pose();
    tool_pose = arm_motion_commander.get_tool_pose_stamped();
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose);
    //alter the tool pose:
    std::cout<<"enter 1: ";
    int ans;
    std::cin>>ans;
    //tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning Cartesian move to goal pose w/ dpx = 0.2"); //, dpy = -0.2");
    //tool_pose.pose.position.y -= 0.2; // move 20cm, along y in torso frame
    tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=arm_motion_commander.plan_path_current_to_goal_gripper_pose(tool_pose);
    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();
    
    //try vector cartesian displacement at fixed orientation:
    ROS_INFO("will plan vertical motion");
    std::cout<<"enter desired delta-z: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    rtn_val = arm_motion_commander.plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=arm_motion_commander.execute_planned_path();
    }
    return 0;
}

