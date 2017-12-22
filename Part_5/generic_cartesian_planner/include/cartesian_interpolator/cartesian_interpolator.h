// cartesian_interpolator 
// wsn, Dec, 2017

#ifndef CARTESIAN_INTERPOLATOR_H_
#define CARTESIAN_INTERPOLATOR_H_

#include<ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace std;

//choose cartesian-path sampling resolution, e.g. 5cm
//const double CARTESIAN_PATH_SAMPLE_SPACING = 0.05; // choose the resolution of samples along Cartesian path
//const double CARTESIAN_PATH_FINE_SAMPLE_SPACING = 0.005; // fine resolution for precision motion

//bool CartesianInterpolator::cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
//        int nsteps,  std::vector<Eigen::Affine3d> &cartesian_affine_samples)

class CartesianInterpolator {
private:
    Eigen::Vector3d n_des_, t_des_, b_des_;
    Eigen::Vector3d n_des_up_, t_des_up_, b_des_up_;
    Eigen::Vector3d tool_n_des_horiz_, tool_t_des_horiz_, tool_b_des_horiz_;

    Eigen::Affine3d a_tool_start_, a_tool_end_;
    Eigen::Matrix3d R_gripper_down_, R_gripper_horiz_, R_gripper_up_;
    std::vector<Eigen::VectorXd> optimal_path_;

    std::vector<Eigen::Affine3d> cartesian_affine_samples_;

    double cartesian_path_sample_spacing_;
    double cartesian_path_fine_sample_spacing_;
public:
    CartesianInterpolator(); //define the body of the constructor outside of class definition

    ~CartesianInterpolator(void) {
    }

    bool cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end,
            int nsteps, std::vector<Eigen::Affine3d> &cartesian_affine_samples);
    bool multipoint_cartesian_path_planner(std::vector<Eigen::Affine3d> a_flange_poses, std::vector<int> nsteps_vec,
            std::vector<Eigen::Affine3d> &cartesian_affine_samples, std::vector<int> &nsteps_to_via_pt);


};

#endif	
