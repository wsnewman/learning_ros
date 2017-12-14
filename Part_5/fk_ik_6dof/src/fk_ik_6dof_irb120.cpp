/* 
 * File:   fk_ik_6dof_irb120.cpp
 * Author: wsn
 *
 * Created Dec, 2017
 */
//this is a "wrapper" that can be used by higher-level code generic for 6DOF robots
// this library is specific to the ABB IRB120 6-DOF robot


#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <fk_ik_6dof/fk_ik_6dof.h>
//const int NJNTS=6; //this library is specific to 6DOF robots

//the following is specific to the IRB120 robot:
#include <irb120_fk_ik/irb120_kinematics.h>
Irb120_fwd_solver irb120_fwd_solver;  //global fk and ik objects
Irb120_IK_solver  irb120_IK_solver;

FwdSolver::FwdSolver() { //constructor

}

Eigen::Affine3d FwdSolver::fwd_kin_solve(Eigen::VectorXd const& q_vec) { // given vector of q angles, compute fwd kin
 Eigen::Affine3d fwd_soln;

 fwd_soln = irb120_fwd_solver.fwd_kin_solve(q_vec);
 return fwd_soln;
}

Eigen::MatrixXd FwdSolver::jacobian(const Eigen::VectorXd& q_vec) {
  Eigen::MatrixXd jacobian;
// ... do some work here FINISH ME!
  return jacobian;
}

IKSolver::IKSolver() {

}

int IKSolver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns) {
  int nsolns = irb120_IK_solver.ik_solve(desired_hand_pose,  q_ik_solns);
  return nsolns;
}


