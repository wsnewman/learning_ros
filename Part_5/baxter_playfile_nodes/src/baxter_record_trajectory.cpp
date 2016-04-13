// baxter_record_trajectory: 
// wsn, April, 2016
// get the robot ready; start this program; enter 1 to start recording
// saves trajectory to files "baxter_r_arm_traj.jsp" and "baxter_l_arm_traj.jsp"

#include<ros/ros.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_recorder_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        
    Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;

    ofstream outfile_right, outfile_left;
    outfile_right.open("baxter_r_arm_traj.jsp");
    outfile_left.open("baxter_l_arm_traj.jsp");


    double dt_samp = 0.2; // sample at 5Hz
    double dt_spin = 0.01;
    double dt_inc = 0.0;
    double arrival_time = 0.0;

    cout << "instantiating a traj streamer" << endl; // enter 1:";
    //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    //get current pose of left and right arms:
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
    cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
    cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

    int ans;
    cout << "enter 1 to start capturing, then move arms in desired trajectory; control-C when done recording: ";
    cin >> ans;
    while (ros::ok()) {

        ros::spinOnce();
        ros::Duration(dt_spin).sleep();
        dt_inc += dt_spin;
        if (dt_inc >= dt_samp) {
            arrival_time += dt_samp;
            dt_inc = 0.0;
            q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
            q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
            //save to disk:
            //outfile << q_in_vecxd.transpose()<<endl;
            outfile_right << q_vec_right_arm[0] << ", " << q_vec_right_arm[1] << ", " << q_vec_right_arm[2]
                    << ", " << q_vec_right_arm[3] << ", " << q_vec_right_arm[4] << ", " << q_vec_right_arm[5]
                    << ", " << q_vec_right_arm[6] << ", " << arrival_time << endl;
            outfile_left << q_vec_left_arm[0] << ", " << q_vec_left_arm[1] << ", " << q_vec_left_arm[2]
                    << ", " << q_vec_left_arm[3] << ", " << q_vec_left_arm[4] << ", " << q_vec_left_arm[5]
                    << ", " << q_vec_left_arm[6] << ", " << arrival_time << endl;
        }
    }

    outfile_right.close();
    outfile_left.close();
    return 0;
}

