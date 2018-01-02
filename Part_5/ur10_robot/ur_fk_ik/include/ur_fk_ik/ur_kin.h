/*********************************************************************
 * Wyatt Newman fk_ik library for UR10 robot
 *********************************************************************/
//test fk/ik with: 
// roslaunch ur_gazebo ur10.launch
// rosrun tf tf_echo base_link tool0
// rosrun rosrun ur_fk_ik ur10_fk_ik_test_main

//HACK:  const double DH_q_max2 = 0; //deg2rad*180; //NOT PHYSICAL LIMIT; IMPOSE TO FORCE ELBOW ELEVATED

#ifndef UR_KIN_H
#define UR_KIN_H

// These kinematics find the transform from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1
//but tool0 frame is same as last DH frame;
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>
const int NJNTS=6;
using namespace std;

//UR10 values:
const double ur10_d1 =  0.1273; //alpha = pi/2
const double ur10_a2 = 0.612; //-0.612; //alpha=0
const double ur10_a3 = 0.5723; //-0.5723; //alpha=0
const double ur10_d4 =  0.163941; //alpha = pi/2; 
const double ur10_d5 =  0.1157; //alpha = pi/2
const double ur10_d6 =  0.0922; //alpha = pi/2
    
const double DH_a1=0.0;
const double DH_a2= ur10_a2; //-0.612;
const double DH_a3= ur10_a3; //-0.5723;
const double DH_a4=0.0;
const double DH_a5=0.0;
const double DH_a6=0.0;



//robot.DH.d='[0.290 0 0 0.302 0 0.072]';
const double DH_d1 = ur10_d1;
const double DH_d2 = 0.0;
const double DH_d3 = 0.0;
const double DH_d4 = ur10_d4;
const double DH_d5 = ur10_d5;
const double DH_d6 = ur10_d6;

//hmm...should alpha1 be +pi/2??
const double DH_alpha1 = -M_PI/2.0; //-M_PI/2.0;
const double DH_alpha2 = 0.0;
const double DH_alpha3 = 0.0;
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = M_PI/2.0; //-M_PI/2.0;
const double DH_alpha6 = 0.0;

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
const double DH_q_offset1 = 0.0;
const double DH_q_offset2 = 0.0; //M_PI; //-M_PI/2.0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = M_PI; //M_PI/2.0; //0.0;
const double DH_q_offset5 = M_PI; //0.0;
const double DH_q_offset6 = M_PI; //0.0;

const double deg2rad = M_PI/180.0;
//actually, can rotate more than this--but simplify
const double DH_q_max1 = deg2rad*180;
const double DH_q_max2 = 0; //deg2rad*180; //NOT PHYSICAL LIMIT; IMPOSE TO FORCE ELBOW ELEVATED
const double DH_q_max3 = deg2rad*180;
const double DH_q_max4 = deg2rad*180;
const double DH_q_max5 = deg2rad*180;
const double DH_q_max6 = deg2rad*180; 

const double DH_q_min1 = -deg2rad*180;
const double DH_q_min2 = -deg2rad*180;
const double DH_q_min3 = -deg2rad*180; 
const double DH_q_min4 = -deg2rad*180;
const double DH_q_min5 = -deg2rad*180;
const double DH_q_min6 = -deg2rad*180; 

const double DH_a_params[]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6};
const double DH_d_params[] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6};
const double DH_alpha_params[] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6};
const double DH_q_offsets[] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6};
const double q_lower_limits[] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6};
const double q_upper_limits[] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6};

const double g_qdot_max_vec[] = {2.16, 2.16, 3.15, 3.2, 3.2, 3.2}; //values per URDF ur10.urdf.xacro in ur_description
const double g_q_home_pose[6] = {0,-1.5707,-1.5707,0,0,0};

class UR10FwdSolver {
    
public:
    UR10FwdSolver(); //constructor; 
    Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Matrix4d get_wrist_frame();
    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
    //Eigen::Matrix3d test_R61(Eigen::VectorXd q_in);
    void q_UR_to_q_DH(Eigen::VectorXd q_soln_UR, Eigen::VectorXd &q_soln_DH);
    void q_DH_to_q_UR(Eigen::VectorXd q_soln_DH, Eigen::VectorXd &q_soln_UR);
    bool fit_joints_to_range(Eigen::VectorXd &qvec);    
    bool fit_q_to_range(double q_min, double q_max, double &q);    
    Eigen::Affine3d get_affine_tool_wrt_flange() { return A_tool_wrt_flange_;}
    void set_affine_tool_wrt_flange(Eigen::Affine3d A_tool_wrt_flange) { 
        A_tool_wrt_flange_=A_tool_wrt_flange;
    }    
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);
private:
    Eigen::Affine3d A_tool_wrt_flange_;
    Eigen::Matrix4d fwd_kin_solve_(const Eigen::VectorXd& q_vec);
    Eigen::Matrix4d A_mats[6], A_mat_products[6], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::MatrixXd Jacobian;    
    
};


class UR10IkSolver:UR10FwdSolver {
public:
    UR10IkSolver(); //constructor; 

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    //int ik_solve(Eigen::Affine3d const& desired_hand_pose); // given vector of q angles, compute fwd kin
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,vector<Eigen::VectorXd> &q_ik_solns);
    //void get_solns(std::vector<Eigen::VectorXd> &q_solns);

    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
private:    
    std::vector<Eigen::VectorXd> q6dof_solns;
    std::vector<Eigen::VectorXd> q_solns_fit;
    Eigen::Matrix4d A_mats[6], A_mat_products[6], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus;
    double L_forearm;
    double phi_elbow;
    //given desired flange pose, compute q1 options; expect 2 or 0; return false if out of reach
    bool compute_q1_solns(Eigen::Vector3d w_des, std::vector<double> &q1_solns);
    void compute_q5_solns(Eigen::Vector3d p_des,std::vector<double> q1_solns, 
        std::vector<double> &q5_solns_1a, std::vector<double> &q5_solns_1b); 
    //in this version, have that R61, third col = [c234*s5; s234*s5; -c5]
    void compute_q5_solns_from_R(Eigen::Matrix3d R61, std::vector<double> &q5_solns);    
    //bool compute_q234_solns(Eigen::Vector3d b61, std::vector<double> q5_solns, 
    //      std::vector<double> &q234_solns);   
    bool compute_q6_solns(Eigen::Matrix3d target_R61, std::vector<double> q5_solns,
         std::vector<double> &q6_solns);
    //double solve_for_theta2(double q1,Eigen::Vector3d w_des);
    //bool solve_q2q3(Eigen::Vector3d w_wrt_1, double q234, vector<double> &q2_solns, vector<double> &q3_solns);
    bool solve_2R_planar_arm_elbow_angs(double x_des, double y_des, double L1, double L2,
         vector<double> &q_elbow_solns);  
    bool solve_2R_planar_arm_shoulder_ang(double x_des,double y_des, double L1, double L2,
           double q_elbow, double &q_shoulder);    
    bool solve_2R_planar_arm(double x_des, double y_des, double L1, double L2,
         vector<double> &q_shoulder_solns,vector<double> &q_elbow_solns);

};

#endif //UR_KIN_H
