// example_generic_cart_move_ac: 
// wsn, Nov, 2016; updated 12/17
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;
    bool traj_is_valid = false;
    int rtn_code;
    
    nsteps = 10;
    arrival_time = 2.0;
    
    //inquire re/ joint angles:
    ROS_INFO("requesting joint angles");
    joint_angles = cart_motion_commander.get_joint_angles();    
    njnts = joint_angles.size();
    cart_motion_commander.set_njnts(njnts);
    
    ROS_INFO("sending test goal");
    cart_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("commanding move to waiting pose");
    //plan_jspace_traj_current_to_waiting_pose(int nsteps, double arrival_time)
    //choose to move in 10 steps over 2 seconds
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_waiting_pose(nsteps,arrival_time);//plan_jspace_traj_current_to_waiting_pose
    
    //send command to execute planned motion
    ROS_INFO("sending execute_planned_path");
    rtn_val=cart_motion_commander.execute_planned_traj();
    ros::Duration(2.0).sleep(); //wait for robot to settle
    
    //inquire re/ joint angles:
    ROS_INFO("requesting joint angles");
    joint_angles = cart_motion_commander.get_joint_angles();
    
    tool_pose_home = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose_home);

    
    //do a joint-space move; get the start angles:
    joint_angles = cart_motion_commander.get_joint_angles();
    njnts = joint_angles.size();
    //increment all of the joint angles by a fixed amt:
    for (int i=0;i<njnts;i++) joint_angles[i]+=0.2;
    ROS_INFO("joint-space move, all joints +0.2 rad");
    //try planning a joint-space motion to this new joint-space pose:
    //plan a traj to specified joint pose using 10 steps and arrival time of 2 seconds
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_qgoal(nsteps, arrival_time, joint_angles);

    //send command to execute planned motion
    rtn_val=cart_motion_commander.execute_planned_traj();   
    ros::Duration(2.0).sleep();
    
    //let's see where we ended up...should match goal request
    joint_angles = cart_motion_commander.get_joint_angles();
    
    tool_pose = cart_motion_commander.get_tool_pose_stamped(); //find tool pose for this jspace pose    
    //return to pre-defined pose by planning path to cartesian pose:
    // send move plan request:


   // bool CartMotionCommander::plan_jspace_traj_qstart_to_des_tool_pose(Eigen::VectorXd  q_start,  int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose){
    //plan_jspace_traj_current_to_tool_pose
    //    bool plan_jspace_traj_current_to_tool_pose(int nsteps, double arrival_time,geometry_msgs::PoseStamped des_pose);   //computes a jspace traj from start pose to some IK soln of desired tool pose

    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose_home);
    //send command to execute planned motion
    rtn_val=cart_motion_commander.execute_planned_traj();    
    ros::Duration(2.0).sleep();    
    
    //now, go back again, using former tool pose as destination and use Cartesian path
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time,tool_pose);   
    //send command to execute planned motion; SHOULD check validity of plan
    rtn_val=cart_motion_commander.execute_planned_traj();    
    ros::Duration(2.0).sleep();   
    
    //get resuling tool pose
    //rtn_val = cart_motion_commander.request_tool_pose();
    tool_pose = cart_motion_commander.get_tool_pose_stamped();
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose);
    /*
    //alter the tool pose:
//    std::cout<<"enter 1: ";
//    int ans;
//    std::cin>>ans; 
    //tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning Cartesian move to goal pose w/ dpx = 0.2"); //, dpy = -0.2");
    //tool_pose.pose.position.y -= 0.2; // move 20cm, along y in torso frame
    tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=cart_motion_commander.plan_path_current_to_goal_gripper_pose(tool_pose);
    //send command to execute planned motion
    rtn_val=cart_motion_commander.execute_planned_traj();
    
    //try vector cartesian displacement at fixed orientation:
    ROS_INFO("will plan vertical motion");
    std::cout<<"enter desired delta-z: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    rtn_val = cart_motion_commander.plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
    }
     * */
    return 0;
}

