// baxter_jnt_ctlr_client_home.cpp: demonstrates use of joint_trajectory_controller interface
// wsn 9/2016
// need to run: rosrun baxter_interface joint_trajectory_action_server.py --mode position
// then run this pgm; 

// this version sends all joints of right arm to zero angles
// The trajectory has only 2 points--and the joint controller must do the interpolation
// node prompts user for move time.  If move time is too short, server will abort and
// will complain: Exceeded Max Goal Velocity Threshold for right arm


#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count = 0;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    //ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "jnt_traj_ctlr_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_goal_right; //, q_pre_pose_left; //this pgm only controls the right arm; don't need "left" vars
    Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
    std::vector<Eigen::VectorXd> des_path_right, des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left; // empty trajectories   

    //here are hard-coded goal joint angles 
    cout << "setting goal pose: " << endl;
    q_goal_right.resize(7);
    q_goal_right << 0, 0, 0, 0, 0, 0, 0;
    //corresponding values to mirror the left arm pose:
    //q_pre_pose_left << 0.907528, 0.111813, -2.06622, 1.8737, 1.295, 2.00164, -2.87179;
    cout << "goal pose right arm: " << q_goal_right.transpose() << endl;

    //ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce(); //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }

    //get current pose of left and right arms:
    //cout << "right arm is at: " << baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose() << endl; 
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
    cout << "right-arm current jnt positions:" << q_vec_right_arm.transpose() << endl;

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
    cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

    //do this manually, as extracted from stuff_trajectory:    
    // only include 2 pts, so trajectory controller must perform the interpolation

    cout << "stuffing traj: " << endl;
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;

    trajectory_msgs::JointTrajectory new_trajectory;
    trajectory_point1.positions.clear();


    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();

    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    trajectory_point1.time_from_start = ros::Duration(0); //first time-from-start is zero
    for (int i = 0; i < 7; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_vec_right_arm[i]);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory

    //2nd pt of trajectory:
    for (int i = 0; i < 7; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_goal_right[i];
    }
    //prompt for an arrival time:
    cout << "enter move duration, in seconds: ";
    double net_time;
    cin >> net_time;
    trajectory_point1.time_from_start = ros::Duration(net_time);
    new_trajectory.points.push_back(trajectory_point1);

    // goal objects compatible with the arm servers
    control_msgs::FollowJointTrajectoryGoal goal_right, goal_left;
    //goal message fields:
    //trajectory_msgs/JointTrajectory trajectory
    //JointTolerance[] path_tolerance
    //JointTolerance[] goal_tolerance
    //duration goal_time_tolerance
    // where [control_msgs/JointTolerance]:
    //string name
    //float64 position
    //float64 velocity
    //float64 acceleration

    //  copy traj to goal:
    goal_right.trajectory = new_trajectory; //
    //goal_left.trajectory = des_trajectory_left;    

    //instantiate client of the right-arm joint-trajectory-control action server:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            right_arm_action_client("robot/limb/right/follow_joint_trajectory", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  


    ROS_INFO("sending goal to right arm: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
    //left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); 
    while (g_done_count < 1) {
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    cout << "right arm is at: " << baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose() << endl;


    return 0;
}

