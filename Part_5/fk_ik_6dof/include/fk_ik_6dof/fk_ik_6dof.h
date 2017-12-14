/* 
 * File:   fk_ik_6dof.h
 * Author: wsn
 *
 * Created Dec, 2017
 */
//this is a "wrapper" header file that can be used by higher-level code generic for 6DOF robots
// examples include: irb120, ur10, abb5400
// abstracts robot specifics and exposes generic commands for fk and ik solvers

#ifndef FK_IK_6DOF_H
#define	FK_IK_6DOF_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>

//const int NJNTS=6;

class FwdSolver {
public:
    FwdSolver(); //constructor
    Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec); // given vector of q angles, compute fwd kin
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);

};

class IKSolver {
public:
    IKSolver(); //constructor; 

    // return the number of valid solutions; 
    // given desired pose, compute IK; return solns in vector of jspace poses
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
};

#endif	/* FK_IK_6DOF_H */

