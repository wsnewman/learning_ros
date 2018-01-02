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
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
           ros::Duration(arrival_time).sleep();
    }
    
    //inquire re/ joint angles:
    ROS_INFO("requesting joint angles");
    joint_angles = cart_motion_commander.get_joint_angles();
    
    tool_pose_home = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose_home);
    //alter the tool pose:
    std::cout<<"enter 1: ";
    int ans;
    std::cin>>ans; 
    
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
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
           ros::Duration(arrival_time).sleep();
    }
    
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
    //int plan_cartesian_traj_qstart_to_des_tool_pose(int nsteps, double arrival_time, Eigen::VectorXd q_start, geometry_msgs::PoseStamped des_pose);
    rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time,tool_pose);   
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
           ros::Duration(arrival_time).sleep();
    }
   
    
    //get resuling tool pose
    //rtn_val = cart_motion_commander.request_tool_pose();
    tool_pose = cart_motion_commander.get_tool_pose_stamped();
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose);

    //alter the tool pose:
//    std::cout<<"enter 1: ";
//    int ans;
//    std::cin>>ans; 
    tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning Cartesian move to goal pose w/ dpz = -0.2"); //, dpy = -0.2");
    //tool_pose.pose.position.y -= 0.2; // move 20cm, along y in torso frame
    //tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    //send command to execute planned motion
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }
    
    ROS_INFO("lower z again and plan  ");
    tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning Cartesian move to goal pose w/ dpz = -0.2"); //, dpy = -0.2")
    //tool_pose.pose.position.y -= 0.2; // move 20cm, along y in torso frame
    //tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    //send command to execute planned motion
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=cart_motion_commander.execute_planned_traj();
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }    
    Eigen::Vector3d b_des,n_des,t_des,O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);
    
    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;
    
    O_des<<0.3,-0.1,0.0;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation()= O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time+0.2).sleep(); 
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }    
    
    //start multi-traj planning:
    int nsegs = 0;
    vector<double> arrival_times;
    
    tool_pose.pose.position.y+=0.2;
     ROS_INFO("requesting plan move along y axis:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    } 
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    
//    int append_multi_traj_cart_segment(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose);
    tool_pose.pose.position.x+=0.1;    
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    tool_pose.pose.position.y-=0.2;
     ROS_INFO("requesting plan move along -y axis:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    //execute multi-traj:
    double wait_time;
    for (int iseg=0;iseg<nsegs;iseg++) {
        ROS_INFO("commanding seg %d",iseg);
        wait_time = arrival_times[iseg];
        rtn_val = cart_motion_commander.execute_traj_nseg(iseg);
        ros::Duration(wait_time).sleep();
    }
    
 
/*
    
    tool_pose = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("resulting tool pose is: ");
    xformUtils.printPose(tool_pose);
    ros::Duration(arrival_time+0.2).sleep();
     tool_pose = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("resulting tool pose is: ");
    xformUtils.printPose(tool_pose);   
   */
/*
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

