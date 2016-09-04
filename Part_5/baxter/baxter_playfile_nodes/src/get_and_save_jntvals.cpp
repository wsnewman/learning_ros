// get_and_save_r_arm_pose: 
// wsn, April, 2016
// uses baxter_traj_streamer to get r_arm and l_arm joint angles; 
// move merry's arms; enter "1" to save to disk, enter "0" to quit the pgm
// saves joint angles to files "baxter_r_arm_angs.txt" and "baxter_l_arm_angs.txt"

#include<ros/ros.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <iostream>
#include <fstream>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;

    ofstream outfile_right, outfile_left;
    outfile_right.open("baxter_r_arm_angs.txt");
    outfile_left.open("baxter_l_arm_angs.txt");
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


    int trigger = 2;
    int npts = 0;
    ROS_INFO("ready to sample and save joint angles to baxter_r_arm_angs.txt and baxter_l_arm_angs.txt");
    while (trigger > 0) {
        cout << "enter 1 for a snapshot, 0 to finish: ";
        cin >> trigger;
        if (trigger == 1) {//take snapshot
            trigger = 2; // reset trigger
            npts++;
            for (int i = 0; i < 10; i++) {
                ros::spinOnce();
                //cout<<"spin "<<i<<endl;
                ros::Duration(0.01).sleep();
            }
            q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
            cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

            q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
            cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;

            //save to disk:
            outfile_right << q_vec_right_arm[0] << ", " << q_vec_right_arm[1] << ", " << q_vec_right_arm[2] << ", "
                    << q_vec_right_arm[3] << ", " << q_vec_right_arm[4] << ", " << q_vec_right_arm[05] << ", "
                    << q_vec_right_arm[6] << ", " << npts << endl;
            outfile_left << q_vec_left_arm[0] << ", " << q_vec_left_arm[1] << ", " << q_vec_left_arm[2] << ", "
                    << q_vec_left_arm[3] << ", " << q_vec_left_arm[4] << ", " << q_vec_left_arm[05] << ", "
                    << q_vec_left_arm[6] << ", " << npts << endl;
        }
    }
    outfile_right.close();
    outfile_left.close();
    return 0;
}

