// ur10_cart_move_as: 
// wsn,  Dec, 2017
// action server to accept commands and perform planning and motion requests
// this code is specific to the ur10 robot, following the pattern of the irb120 robot, 

#include <ur_fk_ik/ur_kin.h> //in this case, choose irb120; change this for different robots

#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
#include <arm_motion_interface/arm_motion_interface.h>
#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>
#include "robot_specific_fk_ik_mappings.h" //SPECIFIC TO TARGET ROBOT
#include "robot_specific_names.h" //THIS MUST BE SPECIFIC TO TARGET ROBOT
#include "planner_joint_weights.h" //need these for joint-space planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10_cart_move_as");
    ros::NodeHandle nh; //standard ros node handle   

    //TEST TEST TEST
    //this is odd...possibly a catkin-simple thing.  Needed to instantiate a CartesianInerpolator to coerce compilation--likely linking oddity
    CartesianInterpolator cartesianInterpolator;

    //TEST TEST TEST
    Eigen::VectorXd q_vec;
    q_vec.resize(6);
    q_vec<<0,0,0,0,0,0;
    //Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec)
    Eigen::Affine3d test_affine;
    test_affine = robotSpecificFK.fwd_kin_solve(q_vec);
    std::cout<<"fwd kin of home pose: origin = "<<test_affine.translation().transpose()<<std::endl;
    ROS_INFO("again...");
    test_affine = pFwdSolver->fwd_kin_solve(q_vec);
    std::cout<<"fwd kin of home pose: origin = "<<test_affine.translation().transpose()<<std::endl;
    

    
    int njnts = g_jnt_names.size();
    ArmMotionInterfaceInits armMotionInterfaceInits;
    armMotionInterfaceInits.urdf_base_frame_name = g_urdf_base_frame_name;
    armMotionInterfaceInits.urdf_flange_frame_name = g_urdf_flange_frame_name;            
    armMotionInterfaceInits.joint_states_topic_name = g_joint_states_topic_name;
    armMotionInterfaceInits.traj_pub_topic_name = g_traj_pub_topic_name;
    armMotionInterfaceInits.jnt_names = g_jnt_names;
    armMotionInterfaceInits.use_trajectory_action_server = g_use_trajectory_action_server;
    armMotionInterfaceInits.traj_as_name = g_traj_as_name;
    armMotionInterfaceInits.pIKSolver_arg = pIKSolver;
    armMotionInterfaceInits.pFwdSolver_arg = pFwdSolver;
    
    for (int i=0;i<njnts;i++) {
       armMotionInterfaceInits.q_lower_limits.push_back(q_lower_limits[i]);
       armMotionInterfaceInits.q_upper_limits.push_back(q_upper_limits[i]);
       armMotionInterfaceInits.qdot_max_vec.push_back(g_qdot_max_vec[i]);
       armMotionInterfaceInits.q_home_pose.push_back(g_q_home_pose[i]);
       armMotionInterfaceInits.planner_joint_weights.push_back(g_planner_joint_weights[i]);
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
