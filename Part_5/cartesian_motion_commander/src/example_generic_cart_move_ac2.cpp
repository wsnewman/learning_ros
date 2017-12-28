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
    

    joint_angles = cart_motion_commander.get_joint_angles();    
    njnts = joint_angles.size();
    cart_motion_commander.set_njnts(njnts);
    
    
    //inquire re/ joint angles:
    ROS_INFO("requesting joint angles");
    joint_angles = cart_motion_commander.get_joint_angles();
    cout<<joint_angles.transpose()<<endl;
    
    tool_pose_home = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose_home);

    return 0;
}

