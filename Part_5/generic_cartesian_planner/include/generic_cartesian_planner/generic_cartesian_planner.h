// cart_path_planner_lib header customized for 6DOF robots
// wsn, Dec, 2017

#ifndef GENERIC_CARTESIAN_PLANNER_H_
#define GENERIC_CARTESIAN_PLANNER_H_

#include<ros/ros.h>

//include the following if/when want to plan a joint-space path and execute it
#include <joint_space_planner/joint_space_planner.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>
//helper class for Cartesian interpolation:
#include <cartesian_interpolator/cartesian_interpolator.h>
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;
//#define VECTOR_DIM 6 // e.g., a 6-dof vector
//const int NJNTS = 6;
//choose cartesian-path sampling resolution, e.g. 5cm
const double CARTESIAN_PATH_SAMPLE_SPACING = 0.05; // choose the resolution of samples along Cartesian path
const double CARTESIAN_PATH_FINE_SAMPLE_SPACING = 0.005; // fine resolution for precision motion

class CartTrajPlanner {
private:
    Eigen::Vector3d n_des_, t_des_, b_des_;
    Eigen::Vector3d n_des_up_, t_des_up_, b_des_up_;
    Eigen::Vector3d tool_n_des_horiz_, tool_t_des_horiz_, tool_b_des_horiz_;

    Eigen::Affine3d a_tool_start_, a_tool_end_;
    Eigen::Matrix3d R_gripper_down_, R_gripper_horiz_, R_gripper_up_;
    std::vector<Eigen::VectorXd> optimal_path_;
    std::vector<Eigen::Affine3d> cartesian_affine_samples;

    IKSolver * pIKSolver_; // instantiate an IK solver
    FwdSolver * pFwdSolver_; //instantiate a forward-kinematics solver   
    std::vector<Eigen::Affine3d> cartesian_affine_samples_;
    // use this class's  fk solver to compute and return tool-flange pose w/rt base, given arm joint angles  
    //Eigen::Affine3d get_fk_Affine_from_qvec(Vectorq7x1 q_vec);
    Eigen::VectorXd jspace_planner_weights_;
    double cartesian_path_sample_spacing_;
    double cartesian_path_fine_sample_spacing_;
    CartesianInterpolator cartesianInterpolator_; //cartesianInterpolator_
    int NJNTS_;
    vector<string> jnt_names_;
   
public:
    CartTrajPlanner(IKSolver * pIKSolver, FwdSolver * pFwdSolver, int njnts); //define the body of the constructor outside of class definition

    ~CartTrajPlanner(void) {
    }
    void set_njnts(int njnts) {NJNTS_ = njnts;};
    void set_jspace_planner_weights(vector<double> planner_joint_weights); //Eigen::VectorXd jspace_planner_weights);
       //{jspace_planner_weights_ = jspace_planner_weights; };
    void set_joint_names(vector<string> jnt_names);
    bool cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::VectorXd> &optimal_path);
    Eigen::Matrix3d get_R_gripper_down(void) {
        return R_gripper_down_;
    }
    bool jspace_trivial_path_planner(Eigen::VectorXd q_start, Eigen::VectorXd q_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool plan_jspace_traj_qstart_to_qend(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &trajectory);

    //bool jspace_path_planner_to_affine_goal(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool plan_jspace_path_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, std::vector<Eigen::VectorXd> &optimal_path);

    //bool plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);
    bool plan_jspace_path_qstart_to_des_flange_affine(Eigen::VectorXd  q_start, int nsteps, Eigen::Affine3d goal_flange_affine,std::vector<Eigen::VectorXd> &optimal_path);
    bool plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);

    bool plan_cartesian_traj_qstart_to_des_flange_affine(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_goal,
        int nsteps,  double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);

    
    bool plan_cartesian_path_w_rot_interp(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::VectorXd> &optimal_path);
    void path_to_traj(std::vector<Eigen::VectorXd> qvecs, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory); 
    //cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, optimal_path_);
    //the next version takes a required start pose in jointspace and plans as above to destination pose
    /*
    bool cartesian_path_planner_w_rot_interp(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end,
            int nsteps, std::vector<Eigen::VectorXd> &optimal_path); */
 /*
    // this following version is the most sophisticated.
    //takes an array of poses and plans a path through them, including orientation interpolation
    //it calls cartesian_path_planner_w_rot_interp() for each segment of the multipoint path, and it appends
    // the resulting IK jspace solutions

    bool multipoint_cartesian_path_planner(std::vector<Eigen::Affine3d> a_flange_poses, std::vector<int> nsteps_vec,
            std::vector<Eigen::VectorXd> &optimal_path, std::vector<int> &nsteps_to_via_pt);

    //as above, but with requirement on first jspace pose:
    //for a_flange_poses, DO start w/ fk(q_start)
    //nsteps_vec[i] specifies number of desired sample points from pose i to pose i+1
    bool multipoint_cartesian_path_planner(Eigen::VectorXd q_start,
            std::vector<Eigen::Affine3d> a_flange_poses, std::vector<int> nsteps_vec,
            std::vector<Eigen::VectorXd> &optimal_path, std::vector<int> &nsteps_to_via_pt);

    bool cartesian_path_planner(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool cartesian_path_planner(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end,
            std::vector<Eigen::VectorXd> &optimal_path, double dp_scalar);
    /// alt version: specify start as a q_vec, and goal as a Cartesian pose (w/rt torso)    
    bool cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path,
            double dp_scalar);
    //better: interpolate both translation and rotation, using angle-axis to interpolate orientation






    /// alt version: specify start as a q_vec, and desired z motion (+ is up) while holding x,y and R fixed
    //bool cartesian_path_planner_zmotion(Vectorq7x1 q_start,double z_dist, std::vector<Eigen::VectorXd> &optimal_path);
    ///alt version: compute path from current pose with cartesian move of delta_p with R fixed
    /// return "true" if successful

    void set_cartesian_path_sample_spacing(double spacing) {
        cartesian_path_sample_spacing_ = spacing;
    };

    void set_cartesian_path_fine_sample_spacing(double spacing) {
        cartesian_path_fine_sample_spacing_ = spacing;
    };

    bool fine_cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);

    bool cartesian_path_planner_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p, std::vector<Eigen::VectorXd> &optimal_path);

    Eigen::Matrix3d get_R_gripper_down(void) {
        return R_gripper_down_;
    }

    Eigen::Matrix3d get_R_gripper_up(void) {
        return R_gripper_up_;
    }
    //void test_IK_solns(std::vector<Eigen::VectorXd > &q_solns);
    void test_IK_solns(std::vector<Eigen::VectorXd> &q_solns);
  * */   
};

#endif	
