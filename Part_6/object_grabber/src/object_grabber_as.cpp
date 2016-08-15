// object_grabber_as: 
// wsn, April, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<geometry_msgs/PoseStamped.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>


#include "arm_motion_commander.cpp" //put class definition in this file
ArmMotionCommander *g_arm_motion_commander_ptr; // a pointer to an ArmMotionCommander object


#include "object_grabber.cpp" // and another class def here

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_grabber_action_server_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    ObjectGrabber object_grabber_as(&nh); // create an instance of the class "ObjectGrabber", containing an action server
    //arm motion commander is convenient means to talk to cartesian motion action server
    ArmMotionCommander arm_motion_commander(&nh);
    g_arm_motion_commander_ptr= &arm_motion_commander; // make this globally accessible; alt: could have object grabber own it
    
        ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }

    return 0;

/*
    Eigen::VectorXd right_arm_joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    geometry_msgs::PoseStamped rt_tool_pose;
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    rtn_val=arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    
    //do a joint-space move; get the start angles:
    right_arm_joint_angles = arm_motion_commander.get_right_arm_joint_angles();
    
    //increment all of the joint angles by a fixed amt:
    for (int i=0;i<7;i++) right_arm_joint_angles[i]+=0.2;
    
    //try planning a joint-space motion to this new joint-space pose:
    rtn_val=arm_motion_commander.rt_arm_plan_jspace_path_current_to_qgoal(right_arm_joint_angles);

    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //return to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();    

    //get tool pose
    rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();
    //alter the tool pose:
    std::cout<<"enter 1: ";
    int ans;
    std::cin>>ans;
    //rt_tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("translating specified dp");
    rt_tool_pose.pose.position.y += 0.5; // move 20cm, along y in torso frame
    rt_tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //try vector cartesian displacement at fixed orientation:
    std::cout<<"enter delta-z: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    }
 * */

}

