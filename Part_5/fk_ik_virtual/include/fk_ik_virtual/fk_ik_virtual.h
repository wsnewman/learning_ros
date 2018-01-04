/* 
 * File:   fk_ik_virtual.h
 * Author: wsn
 *
 * Created Dec, 2017
 */
//this is a "wrapper" header file that can be used by higher-level code generic for 6DOF robots
// examples include: irb120, ur10, abb5400
// abstracts robot specifics and exposes generic commands for fk and ik solvers

#ifndef FK_IK_VIRTUAL_H
#define	FK_IK_VIRTUAL_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>

class FwdSolver {
public:
     FwdSolver() {}; //constructor
    //virtual ~FwdSolver();
    //virtual abc() {} 
    //virtual Eigen::Affine3d fwd_kin_solve() {}; // given vector of q angles, compute fwd kin

    virtual Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec) {}; // given vector of q angles, compute fwd kin
    int dummy();
    virtual Eigen::MatrixXd jacobian(){};
    
    //virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec)=0; // {};

};

class IKSolver {
public:
     IKSolver() {}; //constructor; 
    //virtual ~IKSolver();
    // return the number of valid solutions; 
    // given desired pose, compute IK; return solns in vector of jspace poses
    //virtual int ik_solve(){};
    virtual int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns)=0; //{};
    virtual void ik_refine(std::vector<Eigen::Affine3d> cartesian_affine_samples, std::vector<Eigen::VectorXd> &optimal_path)=0;
};


#endif	/* FK_IK_VIRTUAL_H */

