// example_baxter_cart_move_action_client: 
// wsn, April, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <cartesian_planner/baxter_arm_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_move_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    
    //instantiate a Baxter arm-motion commander object to interface w/ cartMoveActionServer
    ArmMotionCommander arm_motion_commander(&nh);
    
    Eigen::VectorXd right_arm_joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    geometry_msgs::PoseStamped rt_tool_pose, rt_flange_pose;
    
    //illustrate various functions of arm motion commander:
    ROS_INFO("sending a test goal");
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("planning move to pre-pose:");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    ROS_INFO("requesting right-arm joint angles");
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    //ROS_INFO("requesting current right-arm tool pose w/rt torso");
    //rtn_val=arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    ROS_INFO("requesting current right-arm flange pose w/rt torso");
    rtn_val=arm_motion_commander.rt_arm_request_flange_pose_wrt_torso();    
    
    //do a joint-space move; get the start angles:
    right_arm_joint_angles = arm_motion_commander.get_right_arm_joint_angles();
    
    //increment all of the joint angles by a fixed amt:
    ROS_INFO("defining a joint-space move: increment all jnt angles by 0.2 rad");
    for (int i=0;i<7;i++) right_arm_joint_angles[i]+=0.2;
    
    //try planning a joint-space motion to this new joint-space pose:
    ROS_INFO("planning jspace path");
    rtn_val=arm_motion_commander.rt_arm_plan_jspace_path_current_to_qgoal(right_arm_joint_angles);

    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //return to pre-defined pose:
    ROS_INFO("planning path back to pre-pose");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();    

    //get flange pose
    rtn_val = arm_motion_commander.rt_arm_request_flange_pose_wrt_torso();
    rt_flange_pose = arm_motion_commander.get_rt_flange_pose_stamped();
    ROS_INFO("rt_flange_pose origin: %f, %f, %f",rt_flange_pose.pose.position.x,rt_flange_pose.pose.position.y,rt_flange_pose.pose.position.z);
    //alter the tool pose:
    std::cout<<"paused here; enter 1 to start dp move: ";
    int ans;
    std::cin>>ans;
    //rt_tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning cartesian path to move  dp_x = 0.2, dp_y = 0.5");
    
    rt_flange_pose.pose.position.y += 0.5; // move 50cm, along y in torso frame
    rt_flange_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    ROS_INFO("desired pose origin: %f, %f, %f",rt_flange_pose.pose.position.x,rt_flange_pose.pose.position.y,rt_flange_pose.pose.position.z);

    rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_flange_pose(rt_flange_pose);
    //send command to execute planned motion
    ROS_INFO("sending command to execute planned path:");
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //try vector cartesian displacement at fixed orientation:
    std::cout<<"enter desired delta-z motion: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    ROS_INFO("planning path");
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           ROS_INFO("sending command to execute planned path:");
           rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    }
    else {
        ROS_WARN("desired motion is not feasible");
    }
    return 0;
}

