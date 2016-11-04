// ur_traj_client_pre_pose.cpp: 
// wsn 10/2016
// control UR-10 robot with trajectory message;
// start up robot in Gazebo with: roslaunch ur_gazebo ur10.launch
// then run this node (may need to first do: rosparam set use_sim_time true

// 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector
const double dt_traj = 0.02; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
//TEST: deliberately limit joint velocities to very small values
double g_qdot_max_vec[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //put real vel limits here

void set_ur_jnt_names() {
    g_ur_jnt_names.push_back("shoulder_pan_joint");
    g_ur_jnt_names.push_back("shoulder_lift_joint");
    g_ur_jnt_names.push_back("elbow_joint");
    g_ur_jnt_names.push_back("wrist_1_joint");
    g_ur_jnt_names.push_back("wrist_2_joint");
    g_ur_jnt_names.push_back("wrist_3_joint");
}

//need to reference realistic joint velocity limits to compute min transition times
double transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_ur_jnt_names
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
    }

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = 0.05;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<VECTOR_DIM;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }
}


//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_ur_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        cout << "CB: q_vec_right_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}

//action server will respond to this callback when done
void armDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    g_done_move = true;
    //ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur_traj_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    set_ur_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //here are initial, hard-coded joint angles for arm pose
    cout << "setting pre-pose: " << endl;
    q_pre_pose.resize(VECTOR_DIM);
    q_pre_pose << 0, 0, 0, 0, 0, 0; //-0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //get current pose of arm:  
    cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;

    //instantiate client of the arm server:
    //create an action client of UR's follow_joint_trajectory action server 
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            arm_action_client("/arm_controller/follow_joint_trajectory", true);

    // attempt to connect to the server(s):
    ROS_INFO("waiting for arm-control server: ");
    bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  

    int nposes = 6;
    int ans;
    control_msgs::FollowJointTrajectoryGoal goal; //consistent goal message for UR action service

    double q234;
    Eigen::Vector3d bz61;
    for (int i = 0; i < nposes; i++) {
        cout << "enter 1: "; //poor-man's break point
        cin >> ans;
        q_pre_pose[i] = -2.0; //each time through, set successive joint cmd to -2 rad
        des_path.clear();
        des_path.push_back(g_q_vec_arm_Xd); //start from current pose
        des_path.push_back(q_pre_pose); //and go to new desired pose
        stuff_trajectory(des_path, des_trajectory); //convert path to traj
        goal.trajectory = des_trajectory; // and put in a goal message

        ROS_INFO("sending goal to  arm: ");
        arm_action_client.sendGoal(goal, &armDoneCb); //ship off goal to action server
        while (g_done_count < 1) { //wait for "done" flag
            ROS_INFO("waiting to finish move..");
            ros::spinOnce(); //print jnt angles while waiting
            cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
        //q234 = g_q_vec_arm_Xd[1]+g_q_vec_arm_Xd[2]+g_q_vec_arm_Xd[3];
        //ROS_INFO("q234 = %f",q234);        
    }
    int jnt=1;
    double qval;
    while(jnt>=0) {
        cout<<"enter jnt num, 0 through 6: ";
        cin>>jnt;
        cout<<"enter jnt angle: ";
        cin>>qval;
        q_pre_pose[jnt] = qval; //each time through, set successive joint cmd to -2 rad
        des_path.clear();
        des_path.push_back(g_q_vec_arm_Xd); //start from current pose
        des_path.push_back(q_pre_pose); //and go to new desired pose
        stuff_trajectory(des_path, des_trajectory); //convert path to traj
        goal.trajectory = des_trajectory; // and put in a goal message

        ROS_INFO("sending goal to  arm: ");
        arm_action_client.sendGoal(goal, &armDoneCb); //ship off goal to action server
        while (g_done_count < 1) { //wait for "done" flag
            ROS_INFO("waiting to finish move..");
            ros::spinOnce(); //print jnt angles while waiting
            cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;        
    }

    return 0;
}

