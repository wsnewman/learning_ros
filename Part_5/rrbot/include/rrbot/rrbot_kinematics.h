/* 
 * File:   rrbot_kinematics.h
 * Author: wsn
 *
 * Created June, 2016
 */


#ifndef RRBOT_KIN_H
#define	RRBOT_KIN_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>

typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
// a fwd kin solver...
// list DH params here for rrbot:
// 
//world frame has origin on ground plane and z-axis pointing up
//DH frame0 has z-axis through joint1, antiparallel to world-y axis
// subject to this constraint, get choices for orientation and origin of frame0;
// choose frame0 x-axis parallel to world-frame x-axis, 
// and thus frame0 y-axis is parallel to world-frame z-axis;
// choose frame0 origin offset by 1.95 along world z and -0.1 along world y

const int NJNTS=2; //number of degrees of freedom of this robot

const double base_to_frame0_dz = 1.95;
const double base_to_frame0_dy = -0.1;
const double base_to_frame0_dx = 0.0;
const double base_to_frame0_rotx = M_PI/2.0;

const double DH_a1=0.9; //link length: distance from joint1 to joint2 axes
const double DH_a2=0.95; //link length: distance from joint2 axis to flange-z axis

const double DH_d1 = 0.1;// offset along parent z axis from frame0 to frame1
const double DH_d2 = 0.0; // zero offset along parent z axis from frame1 to flange

const double DH_alpha1 = 0; //joint1 axis is parallel to joint2 axis
const double DH_alpha2 = 0; //joint2 axis is parallel to flange z-axis

//could define robot "home" angles different than DH home pose; reconcile with these offsets
const double DH_q_offset1 = 0.0; 
const double DH_q_offset2 = 0.0; 

const double deg2rad = M_PI/180.0;

//can choose to restrict joint ranges:
const double DH_q_max1 = deg2rad*160; 
const double DH_q_max2 = deg2rad*160; 

const double DH_q_min1 = -deg2rad*160; 
const double DH_q_min2 = -deg2rad*160;


const double DH_a_params[NJNTS]={DH_a1,DH_a2}; 
const double DH_d_params[NJNTS] = {DH_d1, DH_d2}; 
const double DH_alpha_params[NJNTS] = {DH_alpha1, DH_alpha2}; 
const double DH_q_offsets[NJNTS] = {DH_q_offset1, DH_q_offset2};
const double q_lower_limits[NJNTS] = {DH_q_min1, DH_q_min2};
const double q_upper_limits[NJNTS] = {DH_q_max1, DH_q_max2};


class Rrbot_fwd_solver {
public:
    Rrbot_fwd_solver(); //constructor
    

//fwd kin fncs: tool flange w/rt base (link1)
    Eigen::Affine3d fwd_kin_flange_wrt_world_solve(Eigen::VectorXd q_vec);

    Eigen::MatrixXd Jacobian(Eigen::VectorXd q_vec);
        
    // these are all w/rt base link
    Eigen::Matrix4d get_frame(int i) {return A_mat_products_[i];};     
    Eigen::Matrix4d get_frame0() {return A_mat_products_[0];};    
    Eigen::Matrix4d get_frame1() {return A_mat_products_[1];};
    Eigen::Matrix4d get_flange_frame() {return A_mat_products_[1];}; //flange frame
    
    
//inner fwd-kin fnc: computes tool-flange frame w/rt base frame
    Eigen::Matrix4d fwd_kin_solve_(Eigen::VectorXd q_vec);
    
    Eigen::Matrix4d A_mats_[NJNTS], A_mat_products_[NJNTS], A_base_link_wrt_world_, A_base_link_wrt_world_inv_; 
};


class Rrbot_IK_solver:Rrbot_fwd_solver  {
public:
    Rrbot_IK_solver(); //constructor; 
    bool fit_q_to_range(double q_min, double q_max, double &q);
    bool solve_for_elbow_ang(Eigen::Vector3d O_flange_wrt_world, std::vector<double> &q_elbow_solns);
    bool solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns);
    bool solve_for_shoulder_ang(Eigen::Vector3d O_flange_wrt_world, double q_elbow, double &q_shoulder);
    int ik_solve(Eigen::Affine3d desired_flange_pose_wrt_base,std::vector<Eigen::Vector2d> &q_solns);

};

#endif	/* RRBOT_KIN_H */

