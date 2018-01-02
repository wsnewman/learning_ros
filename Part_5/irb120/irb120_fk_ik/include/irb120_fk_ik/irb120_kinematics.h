/* 
 * File:   irb120_ik.h
 * Author: wsn
 *
 * Created March 10, 2015
 */

#ifndef IRB120_IK_H
#define	IRB120_IK_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>

typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
//#include <boost/shared_ptr.hpp>
//#include <task_variables/library.h>

const int NJNTS=6;

// a fwd kin solver...
// list DH params here
// these values from Matlab "ARTE" agree with URDF from SWRI
//robot.DH.a='[0 0.270 0.070 0 0 0]';
const double DH_a1=0.0;
const double DH_a2=0.270;
const double DH_a3=0.070;
const double DH_a4=0.0;
const double DH_a5=0.0;
const double DH_a6=0.0;



//robot.DH.d='[0.290 0 0 0.302 0 0.072]';
const double DH_d1 = 0.290;
const double DH_d2 = 0.0;
const double DH_d3 = 0.0;
const double DH_d4 = 0.302;
const double DH_d5 = 0.0;
const double DH_d6 = 0.072;

//robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
const double DH_alpha1 = -M_PI/2.0;
const double DH_alpha2 = 0.0;
const double DH_alpha3 = -M_PI/2.0;
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = -M_PI/2.0;
const double DH_alpha6 = 0.0;

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
const double DH_q_offset1 = 0.0;
const double DH_q_offset2 = -M_PI/2.0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0.0;
const double DH_q_offset5 = 0.0;
const double DH_q_offset6 = M_PI;

const double deg2rad = M_PI/180.0;

const double DH_q_max1 = deg2rad*165;
const double DH_q_max2 = deg2rad*110;
const double DH_q_max3 = deg2rad*70;
const double DH_q_max4 = deg2rad*160;
const double DH_q_max5 = deg2rad*120;
const double DH_q_max6 = deg2rad*180; //deg2rad*400; //deliberately reduce this, to avoid excess solutions

const double DH_q_min1 = -deg2rad*165;
const double DH_q_min2 = -deg2rad*110;
const double DH_q_min3 = -deg2rad*110; //looks odd--not symmetric, but seems to be correct
const double DH_q_min4 = -deg2rad*160;
const double DH_q_min5 = -deg2rad*120;
const double DH_q_min6 = -deg2rad*180; //-deg2rad*400;

const double DH_a_params[]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6};
const double DH_d_params[6] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6};
const double DH_alpha_params[6] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6};
const double DH_q_offsets[6] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6};
const double q_lower_limits[6] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6};
const double q_upper_limits[6] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6};
const double g_qdot_max_vec[] = {4.3, 4.3, 4.3, 5.5, 5.5, 5.5}; //values per URDF 
const double g_q_home_pose[6] = {0,0,0,0,0,0};
//put these in planner_joint_weights.h
//const double jspace_planner_weights[] = {5,5,3,0.5,0.2,0.2}; //default weights for jspace planner (changeable in planner)


class Irb120_fwd_solver {
public:
    Irb120_fwd_solver(); //constructor; //const hand_s& hs, const atlas_frame& base_frame, double rot_ang);
    //atlas_hand_fwd_solver(const hand_s& hs, const atlas_frame& base_frame);
    //Eigen::Affine3d fwd_kin_solve(const Vectorq6x1& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); 
    Eigen::Matrix4d get_wrist_frame();
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);
    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
private:
    Eigen::Matrix4d fwd_kin_solve_(const Vectorq6x1& q_vec);
    Eigen::Matrix4d A_mats[6], A_mat_products[6], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::MatrixXd Jacobian;
};

class Irb120_IK_solver {
public:
    Irb120_IK_solver(); //constructor; 

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    int ik_solve(Eigen::Affine3d const& desired_hand_pose); // given desired pose, compute IK
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
    void get_solns(std::vector<Vectorq6x1> &q_solns);
    bool fit_joints_to_range(Vectorq6x1 &qvec);
    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
private:
    bool fit_q_to_range(double q_min, double q_max, double &q);    
    std::vector<Vectorq6x1> q6dof_solns;
    std::vector<Vectorq6x1> q_solns_fit;
    Eigen::Matrix4d A_mats[6], A_mat_products[6], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus;
    double L_forearm;
    double phi_elbow;
    //given desired flange pose, fill up solns for q1, q2, q3 based on wrist position
    bool compute_q123_solns(Eigen::Affine3d const& desired_hand_pose, std::vector<Vectorq6x1> &q_solns);
    //double solve_for_theta2(double q1,Eigen::Vector3d w_des);
    bool solve_for_theta2(Eigen::Vector3d w_wrt_1,double r_goal, double q2_solns[2]);    
    bool solve_for_theta3(Eigen::Vector3d w_wrt_1,double r_goal, double q3_solns[2]); 

    bool solve_spherical_wrist(Vectorq6x1 q_in,Eigen::Matrix3d R_des, std::vector<Vectorq6x1> &q_solns);    
    //Eigen::MatrixXd Jacobian;
};

/*
//redefine generic names for better compatibility with higher-level code:
//the following merely renames the irb120 solvers to a generic name
class FwdSolver {
public:
    FwdSolver(); //constructor
    Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec); // given vector of q angles, compute fwd kin
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);
    Irb120_fwd_solver irb120_fwd_solver;
};

class IKSolver {
public:
    IKSolver(); //constructor; 
    Irb120_IK_solver irb120_IK_solver;
    // return the number of valid solutions; 
    // given desired pose, compute IK; return solns in vector of jspace poses
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
};
*/
#endif	/* IRB120_IK_H */

