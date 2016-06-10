//test_restore_fk
#include <rrbot/rrbot_kinematics.h> 
#include <sensor_msgs/JointState.h>

Eigen::VectorXd g_q_vec;
using namespace std;

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    for (int i = 0; i < NJNTS; i++) {
        g_q_vec[i] = js_msg.position[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrbot_fk_test");

    ros::NodeHandle nh;
    Eigen::Vector2d q_init;
    q_init<<0,0;
    g_q_vec= q_init; // init global q_vec

    ros::Subscriber joint_state_sub = nh.subscribe("rrbot/joint_states", 1, jointStatesCb);
    Eigen::Affine3d affine_flange;
    //Restore_IK_solver restore_ik_solver;
    Rrbot_fwd_solver rrbot_fwd_solver;
    //Eigen::Vector3d flange_origin;
    //bool valid_q_elbow = false;
    //std::vector<double> q_elbow_solns;
    //q_elbow_solns.clear();
    //double min_solver_err = 100.0;
    while (ros::ok()) {
        cout<<endl<<endl;
        ROS_INFO("angs: %f, %f", g_q_vec[0], g_q_vec[1]);

        affine_flange = rrbot_fwd_solver.fwd_kin_flange_wrt_world_solve(g_q_vec);
        for (int i = 0; i < NJNTS; i++) {
            cout << "frame " << i << " w/rt world: " << endl;
            cout << rrbot_fwd_solver.get_frame(i) << endl;
        }
        //min_solver_err = 100.0;
        cout << "flange origin: " << affine_flange.translation().transpose() << endl;
        cout << "R" << endl;
        cout << affine_flange.linear() << endl;
        Eigen::Quaterniond quat(affine_flange.linear());
        ROS_INFO("quat: %f, %f, %f, %f", quat.x(), quat.y(), quat.z(), quat.w());

        //wrist_pt = restore_fwd_solver.get_wrist_point(g_q_vec);
        //cout << "wrist pt wrt frame0: " << wrist_pt.transpose() << endl;
        //wrist_pt_wrt_frame1 = restore_fwd_solver.get_wrist_coords_wrt_frame1(g_q_vec);
        //cout<<"wrist pt wrt frame1: "<<wrist_pt_wrt_frame1.transpose()<<endl; 
        //q_elbow_solns.clear();
        //valid_q_elbow = restore_ik_solver.solve_for_elbow_ang(wrist_pt, g_q_vec[0], q_elbow_solns);
        /*
        cout << "q_elbow actual = " << g_q_vec[3] << endl;
        cout << "q humerus actual = " << g_q_vec[2] << endl;
        double w_z_1 = DH_d5 * sin(g_q_vec[3]) * sin(g_q_vec[2]) + DH_d4 * cos(g_q_vec[2]) + DH_d2;
        //cout << "w_z_1 from known angles = " << w_z_1 << endl;
        if (!valid_q_elbow) cout << "found no viable q_elbow solns" << endl;
        std::vector<double> q_humerus_angs;
        std::vector<double> q_shoulder_pitch_angs;
        bool valid_q_humerus = false;
        bool valid_q_shoulder = false;
        double q_yaw, q_elbow, q_humerus, q_shoulder;
        q_yaw = g_q_vec[0]; //this angle to be given (indexed in search)
        if (q_elbow_solns.size() > 0) {
            cout << "num elbow solns = " << q_elbow_solns.size() << endl;
            q_elbow = q_elbow_solns[0];
            cout << "q_elbow_soln_a = " << q_elbow << endl;
            q_vec_test[3] = q_elbow; //test w/ this elbow soln

            //solve_for_humerus_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_elbow, std::vector<double> &q_humerus_solns)
            q_humerus_angs.clear();
            valid_q_humerus = restore_ik_solver.solve_for_humerus_ang(wrist_pt, q_yaw, q_elbow, q_humerus_angs);
            if (!valid_q_humerus) cout << "No valid humerus angles found" << endl;
            else {
                q_humerus = q_humerus_angs[0];
                cout << "q_humerus_soln_a=" << q_humerus << endl;
                q_shoulder_pitch_angs.clear();
                //solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus, double q_elbow, std::vector<double> &q_shoulder_solns)
                valid_q_shoulder = restore_ik_solver.solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                if (!valid_q_shoulder) cout << "No valid shoulder angles" << endl;
                else { //test solns w/ FK:
                    cout << "found " << q_shoulder_pitch_angs.size() << " shoulder solns" << endl;
                    q_shoulder = q_shoulder_pitch_angs[0];
                    q_vec_test[0] = q_yaw;
                    q_vec_test[1] = q_shoulder;
                    q_vec_test[2] = q_humerus;
                    q_vec_test[3] = q_elbow;
                    wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                    cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                    w_soln_err = wrist_pt - wrist_pt_test_soln;
                    ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                    cout << w_soln_err.transpose() << endl;
                    cout << "q: " << q_vec_test.transpose() << endl;
                    q_vec_err = g_q_vec - q_vec_test;
                    if (q_vec_err.norm() < min_solver_err) {
                        min_solver_err = q_vec_err.norm();
                    }

                    cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                            << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                    //test 2nd shoulder soln:
                    if (q_shoulder_pitch_angs.size() > 1) {
                        q_shoulder = q_shoulder_pitch_angs[1];
                        q_vec_test[1] = q_shoulder;
                        wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                        cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                        cout << w_soln_err.transpose() << endl;
                        cout << "q: " << q_vec_test.transpose() << endl;
                        q_vec_err = g_q_vec - q_vec_test;
                        if (q_vec_err.norm() < min_solver_err) {
                            min_solver_err = q_vec_err.norm();
                        }
                        cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                    }
                }
                if (q_humerus_angs.size() > 1) {
                    q_humerus = q_humerus_angs[1];
                    q_vec_test[2] = q_humerus;
                    cout << endl << "q_humerus_soln_b=" << q_humerus << endl;
                    valid_q_shoulder = restore_ik_solver.solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                    if (!valid_q_shoulder) cout << "q_humerus soln2: No valid shoulder angles" << endl;
                    else { //test solns w/ FK:               
                        q_shoulder = q_shoulder_pitch_angs[0];
                        q_vec_test[1] = q_shoulder;
                        wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                        cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                        cout << w_soln_err.transpose() << endl;
                        cout << "q: " << q_vec_test.transpose() << endl;
                        q_vec_err = g_q_vec - q_vec_test;
                        if (q_vec_err.norm() < min_solver_err) {
                            min_solver_err = q_vec_err.norm();
                        }
                        cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;

                        //test 2nd shoulder soln:
                        if (q_shoulder_pitch_angs.size() > 1) {
                            q_shoulder = q_shoulder_pitch_angs[1];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                            cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                            ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                            cout << w_soln_err.transpose() << endl;

                            cout << "q: " << q_vec_test.transpose() << endl;
                            q_vec_err = g_q_vec - q_vec_test;
                            if (q_vec_err.norm() < min_solver_err) {
                                min_solver_err = q_vec_err.norm();
                            }
                            cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                    << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                        }
                    }
                }
            }
            if (q_elbow_solns.size() > 1) {
                q_elbow = q_elbow_solns[1];
                cout << "q_elbow_soln_b = " << q_elbow << endl;
                q_humerus_angs.clear();
                valid_q_humerus = restore_ik_solver.solve_for_humerus_ang(wrist_pt, g_q_vec[0], q_elbow, q_humerus_angs);
                if (!valid_q_humerus) cout << "No valid humerus angles found" << endl;
                else {
                    q_humerus = q_humerus_angs[0];
                    cout << "q_humerus_soln_a=" << q_humerus << endl;
                    q_shoulder_pitch_angs.clear();
                    //solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus, double q_elbow, std::vector<double> &q_shoulder_solns)
                    valid_q_shoulder = restore_ik_solver.solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                    if (!valid_q_shoulder) cout << "No valid shoulder angles" << endl;
                    else { //test solns w/ FK:
                        cout << "found " << q_shoulder_pitch_angs.size() << " shoulder solns" << endl;
                        q_shoulder = q_shoulder_pitch_angs[0];
                        q_vec_test[0] = q_yaw;
                        q_vec_test[1] = q_shoulder;
                        q_vec_test[2] = q_humerus;
                        q_vec_test[3] = q_elbow;
                        wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                        cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                        cout << w_soln_err.transpose() << endl;
                        cout << "q: " << q_vec_test.transpose() << endl;
                        q_vec_err = g_q_vec - q_vec_test;
                        if (q_vec_err.norm() < min_solver_err) {
                            min_solver_err = q_vec_err.norm();
                        }
                        cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                        //test 2nd shoulder soln:
                        if (q_shoulder_pitch_angs.size() > 1) {
                            q_shoulder = q_shoulder_pitch_angs[1];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                            cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                            ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                            cout << w_soln_err.transpose() << endl;
                            cout << "q: " << q_vec_test.transpose() << endl;
                            q_vec_err = g_q_vec - q_vec_test;
                            if (q_vec_err.norm() < min_solver_err) {
                                min_solver_err = q_vec_err.norm();
                            }
                            cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                    << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                        }
                    }
                    if (q_humerus_angs.size() > 1) {
                        q_humerus = q_humerus_angs[1];
                        cout << "q_humerus_soln_b=" << q_humerus << endl;
                        q_vec_test[2] = q_humerus;
                        q_shoulder_pitch_angs.clear();
                        valid_q_shoulder = restore_ik_solver.solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                        if (!valid_q_shoulder) cout << "q_humerus soln2: No valid shoulder angles" << endl;
                        else { //test solns w/ FK:               
                            q_shoulder = q_shoulder_pitch_angs[0];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                            cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                            ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                            cout << w_soln_err.transpose() << endl;
                            cout << "q: " << q_vec_test.transpose() << endl;
                            q_vec_err = g_q_vec - q_vec_test;
                            if (q_vec_err.norm() < min_solver_err) {
                                min_solver_err = q_vec_err.norm();
                            }
                            cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                    << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                            //test 2nd shoulder soln:
                            if (q_shoulder_pitch_angs.size() > 1) {
                                q_shoulder = q_shoulder_pitch_angs[1];
                                q_vec_test[1] = q_shoulder;
                                wrist_pt_test_soln = restore_fwd_solver.get_wrist_point(q_vec_test);
                                cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                                w_soln_err = wrist_pt - wrist_pt_test_soln;
                                ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                                cout << w_soln_err.transpose() << endl;
                                cout << "q: " << q_vec_test.transpose() << endl;
                                q_vec_err = g_q_vec - q_vec_test;
                                if (q_vec_err.norm() < min_solver_err) {
                                    min_solver_err = q_vec_err.norm();
                                }
                                cout << "q_err: " << g_q_vec[0] - q_vec_test[0] << ", " << g_q_vec[1] - q_vec_test[1]
                                        << ", " << g_q_vec[2] - q_vec_test[2] << ", " << g_q_vec[3] - q_vec_test[3] << endl;
                            }
                        }

                    }
                }
            }
        }
        ROS_WARN("min solver err = %f", min_solver_err);
         * */
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
}
