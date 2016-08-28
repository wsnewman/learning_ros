// traj_action_client_pre_pose: 
// uses right and left arm trajectory action servers to send robot to a hard-coded pre pose

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include<baxter_trajectory_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count=0;
void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
        g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_pre_pose_right,q_pre_pose_left;
    Eigen::VectorXd q_vec_right_arm,q_vec_left_arm;
    std::vector<Eigen::VectorXd> des_path_right, des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory_right,des_trajectory_left; // empty trajectories     
    
    //here are hard-coded joint angles for left and right arm poses
    cout<<"setting pre-poses: "<<endl;
    q_pre_pose_right.resize(7);
    q_pre_pose_left.resize(7);
    q_pre_pose_right << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0; 
    //q_pre_pose_right << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;   
    //corresponding values to mirror the left arm pose:
    q_pre_pose_left  <<  0.907528, 0.111813, -2.06622, 1.8737, 1.295, 2.00164, -2.87179;
    cout<<"pre-pose right: "<<q_pre_pose_right.transpose()<<endl;
    //cout<<"enter 1";
    //int ans;
    //cin>>ans;
    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();  //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }

    //get current pose of left and right arms:
    cout<<"right arm is at: "<<baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose()<<endl;
    //cout<<"enter 1";
    //cin>>ans;    
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd(); 
    cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd(); 
    cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;
    
    des_path_right.push_back(q_vec_right_arm); //start from current pose
    des_path_right.push_back(q_pre_pose_right); 
    
    des_path_left.push_back(q_vec_left_arm);
    des_path_left.push_back(q_pre_pose_left); 

    cout << "stuffing traj: " << endl;
    //convert from vector of 7dof poses to trajectory message  
    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
    baxter_traj_streamer.stuff_trajectory_left_arm(des_path_left, des_trajectory_left); 
    
    
    // goal objects compatible with the arm servers
    baxter_trajectory_streamer::trajGoal goal_right,goal_left;
    //  copy traj to goal:
    goal_right.trajectory = des_trajectory_right;
    goal_left.trajectory = des_trajectory_left;    

    //instantiate clients of the two arm servers:
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client("leftArmTrajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

    ROS_INFO("waiting for left-arm server: ");
    server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on left-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to left-arm action server"); // if here, then we connected to the server; 

    ROS_INFO("sending goals to left and right arms: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); 
    left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); 
    while (g_done_count<2) {
       ROS_INFO("waiting to finish pre-pose..");
       ros::Duration(1.0).sleep();
    }
    
            ros::spinOnce();
            cout<<"right arm is at: "<<baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose()<<endl;

    
    return 0;
}

