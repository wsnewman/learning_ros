//arm7dof_fk_ik_test_main
// wsn July, 2016
//run this pgm to test the forward and inverse kinematics library for the arm7dof robot


#include <arm7dof_fk_ik/arm7dof_kinematics.h> 
#include <sensor_msgs/JointState.h>
Vectorq7x1 g_q_vec;
using namespace std;

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    for (int i = 0; i < 7; i++) {
        g_q_vec[i] = js_msg.position[i];
    }
}

double q123_err(Eigen::VectorXd q1, Eigen::VectorXd q2) {
    double esqd = 0.0;
    for (int i = 0; i < 4; i++) {
        esqd += (q1(i) - q2(i))*(q1(i) - q2(i));
    }
    return sqrt(esqd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm7dof_fk_ik_test");

    ros::NodeHandle nh;
    g_q_vec << 0, 0, 0, 0, 0, 0, 0;
    Vectorq7x1 q_vec_test, q_vec_err;
    double q_fit_best = 100.0;
    double q_fit_err;
    q_vec_test << 0, 0, 0, 0, 0, 0, 0;
    ros::Subscriber joint_state_sub = nh.subscribe("arm7dof/joint_states", 1, jointStatesCb);
    Eigen::Affine3d affine_flange, affine_test, affine_prod;
    Eigen::Matrix3d R_flange;
    Arm7dof_IK_solver arm7dof_ik_solver;
    Arm7dof_fwd_solver arm7dof_fwd_solver;
    Eigen::Vector3d wrist_pt_wrt_frame1, wrist_pt, wrist_pt_test_soln, w_soln_err;
    bool valid_q_elbow = false;
    std::vector<Eigen::VectorXd> q_solns;

    std::vector<Vectorq7x1> q_solns_7dof;
    double q_yaw;
    int n7dof_solns;
    while (ros::ok()) {
        ROS_INFO("angs: %f, %f, %f, %f, %f, %f, %f", g_q_vec[0], g_q_vec[1], g_q_vec[2], g_q_vec[3],
                g_q_vec[4], g_q_vec[5], g_q_vec[6]);
        q_vec_test = g_q_vec; //provide correct angles...and substitute solns for q2, q3, q4
        q_yaw = g_q_vec[0];

        affine_flange = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(g_q_vec);
        cout << "flange origin: " << affine_flange.translation().transpose() << endl;
        cout << "R" << endl;
        cout << affine_flange.linear() << endl;
        Eigen::Quaterniond quat(affine_flange.linear());
        ROS_INFO("quat: %f, %f, %f, %f", quat.x(), quat.y(), quat.z(), quat.w());


        cout << "calling ik_solve_given_qs0" << endl;
        arm7dof_ik_solver.ik_solve_given_qs0(affine_flange, q_yaw, q_solns_7dof);
        int n7dof_solns = q_solns_7dof.size();
        cout << "num solns found = " << n7dof_solns << endl;
        q_fit_best = 100.0;
        for (int isoln = 0; isoln < n7dof_solns; isoln++) {
            q_vec_test = q_solns_7dof[isoln];
            cout << "q_test: " << q_vec_test.transpose() << endl;
            q_fit_err = (q_vec_test - g_q_vec).norm();
            cout << "q_fit_err = " << q_fit_err << endl;
            if (q_fit_err < q_fit_best) q_fit_best = q_fit_err;
            //test fk of every soln:
            affine_test = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(q_vec_test);
            affine_prod = affine_test.inverse() * affine_flange;
            cout << "R_prod: " << endl;
            cout << affine_prod.linear() << endl;
            cout << "origin_err.norm(): " << affine_prod.translation().norm() << endl;
            int ans;
            cout << "enter 1: ";
            cin >> ans;
        }
        cout << "q_fit_best = " << q_fit_best << endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
}
