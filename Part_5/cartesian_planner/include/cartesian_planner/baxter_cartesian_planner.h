// cart_path_planner_lib header
// wsn, June, 2015
// a library of arm-motion planning functions
#ifndef CARTESIAN_PLANNER_H_
#define CARTESIAN_PLANNER_H_

#include<ros/ros.h>
#include <baxter_fk_ik/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <joint_space_planner/joint_space_planner.h>
#include <Eigen/Eigen>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

//choose cartesian-path sampling resolution, e.g. 5cm
const double CARTESIAN_PATH_SAMPLE_SPACING = 0.05; // choose the resolution of samples along Cartesian path
const double CARTESIAN_PATH_FINE_SAMPLE_SPACING = 0.005; // fine resolution for precision motion

class CartTrajPlanner {
private:

    //ros::NodeHandle nh_; // if need a node handle, get one upon instantiation


    Eigen::Vector3d n_des_, t_des_, b_des_;
    Eigen::Vector3d tool_n_des_horiz_, tool_t_des_horiz_, tool_b_des_horiz_;

    Eigen::Affine3d a_tool_start_, a_tool_end_;
    Eigen::Matrix3d R_gripper_down_, R_gripper_horiz_;
    std::vector<Eigen::VectorXd> optimal_path_;
    std::vector<Eigen::Affine3d> cartesian_affine_samples_;

    Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver_; //instantiate a forward-kinematics solver   

    Eigen::VectorXd jspace_planner_weights_;


    // use this classes baxter fk solver to compute and return tool-flange pose w/rt torso, given right-arm joint angles  
    Eigen::Affine3d get_fk_Affine_from_qvec(Vectorq7x1 q_vec);

public:
    CartTrajPlanner(); //define the body of the constructor outside of class definition

    ~CartTrajPlanner(void) {
    }
    //these planners assume Affine args are right-arm flange w/rt torso
    ///specify start and end poses w/rt torso.  Only orientation of end pose will be considered; orientation of start pose is ignored
    bool cartesian_path_planner(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    /// alt version: specify start as a q_vec, and goal as a Cartesian pose (w/rt torso)    
    bool cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path,
            double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING);
    ///this version uses a small Cartesian step size and refined IK
    bool fine_cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    /// fnc to fine-tune each sample pt of a jspace soln to get more precise matches to desired Cartesian sample points
    bool refine_cartesian_path_plan(std::vector<Eigen::VectorXd> &optimal_path);

    /// alt version--only plan wrist-point motion; don't worry about wrist orientation
    bool cartesian_path_planner_wrist(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);

    bool jspace_trivial_path_planner(Vectorq7x1 q_start, Vectorq7x1 q_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool jspace_path_planner_to_affine_goal(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    ///alt version: compute path from current pose with cartesian move of delta_p with R fixed
    /// return "true" if successful
    bool cartesian_path_planner_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p, std::vector<Eigen::VectorXd> &optimal_path);

    Eigen::Matrix3d get_R_gripper_down(void) {
        return R_gripper_down_;
    }

};

#endif	