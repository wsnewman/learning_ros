// baxter_jnt_ctlr_client.cpp: 
// wsn 9/2016
// control Baxter's arms using the ROS joint_trajectory_controller interface
// need to run: rosrun baxter_interface joint_trajectory_action_server.py --mode position

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <baxter_fk_ik/baxter_kinematics.h> 
#include <Eigen/QR>    

//#include<baxter_trajectory_streamer/trajAction.h> //old, home-brew interpolator message
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count = 0;
int g_done_move = true;


void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    g_done_move = true;
    //ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "jnt_traj_ctlr_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        
    Baxter_fwd_solver baxter_fwd_solver;
    Eigen::VectorXd dp_des,dq_des,q_des;
    Eigen::VectorXd q_pre_pose_right, q_pre_pose_left;
    Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
    std::vector<Eigen::VectorXd> des_path_right, des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left; // empty trajectories   
    Eigen::MatrixXd Jacobian,Jacobian_inv;
    
    // use the ROS-defined control_msgs message
    control_msgs::FollowJointTrajectoryGoal goal_right, goal_left;    
   
    //here is where we (manually) define the desired Cartesian motion
    // specify an incremental motion from dp_des, and specify how many steps to take:
    int nsteps = 200; //200 steps at 1mm is motion of 0.2m
    dp_des.resize(6);
    dp_des<<0.001,0,0,0,0,0; //this is 1mm increments in the selected direction
    dq_des.resize(7);
    q_des.resize(7);
    
    //here are hard-coded joint angles for left and right arm poses
    cout << "setting pre-poses: " << endl;
    q_pre_pose_right.resize(7);
    q_pre_pose_left.resize(7);
    q_pre_pose_right << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;
    //q_pre_pose_right << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;   
    //corresponding values to mirror the left arm pose:
    q_pre_pose_left << 0.907528, 0.111813, -2.06622, 1.8737, 1.295, 2.00164, -2.87179;
    cout << "pre-pose right: " << q_pre_pose_right.transpose() << endl;

    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce(); //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }

    //get current pose of left and right arms:  
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
    cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
    cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;
/*
    des_path_right.push_back(q_vec_right_arm); //start from current pose
    des_path_right.push_back(q_pre_pose_right);

    des_path_left.push_back(q_vec_left_arm); //and end at hard-coded goal pose
    des_path_left.push_back(q_pre_pose_left);

    cout << "stuffing traj: " << endl;
    //convert from vector of 7dof poses to trajectory messages  
    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right);
    baxter_traj_streamer.stuff_trajectory_left_arm(des_path_left, des_trajectory_left);


    // instead of using our custom baxter_trajectory_streamer message...
    //baxter_trajectory_streamer::trajGoal goal_right,goal_left;


    //  copy trajectories to goal messages:
    goal_right.trajectory = des_trajectory_right;
    goal_left.trajectory = des_trajectory_left;

    //instantiate clients of the two arm servers:
    //previously, used our home-grown interpolation action servers
    //actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
    //actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client("leftArmTrajActionServer", true);
*/
    //instead, use Baxter's follow_joint_trajectory action server (right arm shown here)
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> right_arm_action_client("robot/limb/right/follow_joint_trajectory", true);

    // attempt to connect to the server(s):
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

/*
    ROS_INFO("sending goal to right arm: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
    //left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); 
    while (g_done_count < 1) { //changed count test to "1", since testing w/ right-arm only
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }
*/
    ros::spinOnce();
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
    cout << "right arm is at: " << q_vec_right_arm.transpose() << endl;
    q_des = q_vec_right_arm;
    //start from initial pose:
    des_path_right.clear();
    des_path_right.push_back(q_des);
    //compute a vertical path using Jacobian:
    //dp_z was set above to 1mm; do this 100 times;
    Jacobian = baxter_fwd_solver.compute_Jacobian(q_vec_right_arm);
    cout<<"Jacobian is: "<<endl;
    cout<<Jacobian<<endl;
    //have that dx = J*dq; try pseudo inv
    // J'*dx = J'*J*dq
    // (J'*J)_inv*J'*dx = dq
    Eigen::MatrixXd Jtrans_J, Jtrans_J_inv,Jtrans;
    Jtrans = Jacobian.transpose();
    Jtrans_J = Jtrans*Jacobian;
    Jtrans_J_inv = Jtrans_J.inverse();
    dq_des = Jtrans_J_inv*Jtrans*dp_des;
    q_des=q_vec_right_arm;
    for (int i=0;i<nsteps;i++) {
        Jacobian = baxter_fwd_solver.compute_Jacobian(q_des);
        Jtrans = Jacobian.transpose();
        Jtrans_J = Jtrans*Jacobian;
        Jtrans_J_inv = Jtrans_J.inverse();
        dq_des = Jtrans_J_inv*Jtrans*dp_des;
        q_des = q_des + dq_des;  
        des_path_right.push_back(q_des);     
    }

    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right);
    goal_right.trajectory = des_trajectory_right;
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
    g_done_move  = false;
    while (!g_done_move) { //changed count test to "1", since testing w/ right-arm only
        ROS_INFO("waiting to finish Jacobian move...");
        ros::Duration(0.5).sleep();
    }    
    ROS_INFO("done w/ Jacobian move");
    return 0;
}

