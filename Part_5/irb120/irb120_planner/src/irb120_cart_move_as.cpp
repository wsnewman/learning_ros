// irb120_cart_move_as: 
// wsn,  Dec, 2017
// action server to accept commands and perform planning and motion requests
// this code is specific to the irb120, 
// it includes a definition and implementaton for ArmMotionInterface, used in the action server



//NOTE: ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE does: unpack_goal_pose(),
// which does:     a_flange_end_ = a_tool_end_*A_tool_wrt_flange_.inverse();
//  ik functions are w/rt desired flange frame

// uses library of arm-motion planning functions
//#include <cartesian_planner/irb120_cartesian_planner.h>
//#include <arm_motion_interface/arm_motion_interface.h>
/*
#include <cartesian_planner/cart_moveAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <ur_fk_ik/ur_kin.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/JointState.h>
//#include<moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
*/


//#include <arm_motion_interface/arm_motion_interface.h>
#include <irb120_fk_ik/irb120_kinematics.h> //in this case, choose irb120; change this for different robots
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
#include <arm_motion_interface/arm_motion_interface.h>
#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>
#include "robot_specific_fk_ik_mappings.h" //SPECIFIC TO TARGET ROBOT
#include "robot_specific_names.h" //THIS MUST BE SPECIFIC TO TARGET ROBOT

int main(int argc, char** argv) {
    ros::init(argc, argv, "irb120_cart_move_as");
    ros::NodeHandle nh; //standard ros node handle   

    //TEST TEST TEST
    //this is odd...possibly a catkin-simple thing.  Needed to instantiate a CartesianInerpolator to coerce compilation--likely linking oddity
    CartesianInterpolator cartesianInterpolator;
    
    int njnts = g_jnt_names.size();
    ArmMotionInterfaceInits armMotionInterfaceInits;
    armMotionInterfaceInits.urdf_base_frame_name = g_urdf_base_frame_name;
    armMotionInterfaceInits.urdf_flange_frame_name = g_urdf_flange_frame_name;            
    armMotionInterfaceInits.joint_states_topic_name = g_joint_states_topic_name;
    armMotionInterfaceInits.traj_pub_topic_name = g_traj_pub_topic_name;
    armMotionInterfaceInits.jnt_names = g_jnt_names;
    armMotionInterfaceInits.pIKSolver_arg;
    armMotionInterfaceInits.pFwdSolver_arg;
    
    for (int i=0;i<njnts;i++) {
       armMotionInterfaceInits.q_lower_limits.push_back(q_lower_limits[i]);
       armMotionInterfaceInits.q_upper_limits.push_back(q_upper_limits[i]);
       armMotionInterfaceInits.qdot_max_vec.push_back(g_qdot_max_vec[i]);
       armMotionInterfaceInits.q_home_pose.push_back(g_q_home_pose[i]);
    }
     
            
    ROS_INFO("instantiating an ArmMotionInterface");
    ArmMotionInterface armMotionInterface(&nh,armMotionInterfaceInits);    
     
    //geometry_msgs::PoseStamped toolframe_pose;
     
    //ros::Publisher toolframe_pose_publisher= nh.advertise<geometry_msgs::PoseStamped>("toolframe_pose", 1, true);   
     
    // start servicing requests:
    ROS_INFO("ready to start servicing cartesian-space goals");
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep(); //don't consume much cpu time if not actively working on a command
        //if paint spray is on, get and publish the painthead pose
        /*
        if (g_spray_on) {
          armMotionInterface.compute_tool_stamped_pose_wrt_world();
          painthead_pose = armMotionInterface.get_current_gripper_stamped_pose_wrt_world();
          toolframe_pose_publisher.publish(painthead_pose);
        }
         * */
    }

    return 0;
}
