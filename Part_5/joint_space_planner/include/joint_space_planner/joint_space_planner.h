// wsn, October, 2014
// joint-space planner, organized as a class

#ifndef JOINT_SPACE_PLANNER_H
#define	JOINT_SPACE_PLANNER_H
#include <iostream>
#include <ros/ros.h>

//#include "/usr/include/eigen3/Eigen/Core"
//#include <Eigen/Eigen>
//#include <Eigen/Dense>
#include <Eigen/Core>
//#include <Eigen/LU>

using namespace std;

class JointSpacePlanner {
private:
    // will use convention: trailing underscore ("_") indicates member variable or method
    Eigen::VectorXd penalty_weights_;
    std::vector<std::vector<Eigen::VectorXd> > path_options_; 
    std::vector<std::vector<Eigen::VectorXd> > *path_options_ptr_;     
    std::vector<std::vector<double> > all_costs_; // need to resize these appropriate to problem size, at run time
    std::vector<std::vector<int> > next_indices_;   
    Eigen::VectorXd vec_of_zeros_;
  
    std::vector<Eigen::VectorXd> optimal_path_; // this is a sequence of joint-space poses defining a path

    std::vector<Eigen::VectorXd> prior_pose_options_;
    Eigen::VectorXd target_pose_;
    Eigen::VectorXd prior_pose_; // a single pose to evaluate
    std::vector<Eigen::VectorXd> next_pose_options_;   //an entire "layer" of next-pose options to evaluate
    std::vector<double> target_pose_costs_to_go_;   //an entire "layer" of next-pose options to evaluate
    double min_cost_to_go_;
    int move_index_min_cost_to_go_;
    int problem_dimension_; // dimension of pose options being considered;  e.g., 8dof state-space pose
    int nlayers_; // number of "layers" in path_options; e.g., a "layer" may corresponding to a nominal tool pose,for which there are many IK options
    Eigen::VectorXd diff_;
    Eigen::VectorXd diff_sqd_;    
    double min_total_trip_cost_;
    void constructor_helper_();
    void augment_all_min_costs_(vector<vector<double> > &humerus_sensitivities);
    
public:    
    JointSpacePlanner(std::vector<std::vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights); // option to provide weights
    //alternative constructor--uses humerus sensitivites in cost function
    JointSpacePlanner(vector<vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights, vector<vector<double> > &humerus_sensitivities);
    
    JointSpacePlanner(int i, int j); // dummy test constructor
    
    double score_move(Eigen::VectorXd pose1, Eigen::VectorXd pose2); // compute incremental cost to go from pose1 to pose2, weighted, possibly squared
    bool find_best_moves_single_layer(int target_layer_index); //compute optimal choices for transitions to layer target_layer_index
    bool find_best_moves_single_layer(vector<vector<Eigen::VectorXd> > &path_options,int target_layer_index);
    bool compute_all_min_costs();
    bool compute_all_min_costs(vector<vector<Eigen::VectorXd> > &path_options);    
    bool find_best_move(int ipose,int jlayer);  // for layer "j", choose pose-option "i" and find the index of the lowest cost-to-go to advance to next layer
    // here's the main function: given the pose options at each "layer" (from constructor), find the optimal joint-space path through the layers
    // fill in the answer in optimal_path  
    bool compute_optimal_path(std::vector<Eigen::VectorXd> optimal_path);
    bool compute_optimal_path(vector<vector<Eigen::VectorXd> > &path_options);
    void get_soln(std::vector<Eigen::VectorXd> &optimal_path); // copy solution in to provided container, "optimal_path"
    double get_trip_cost() { return min_total_trip_cost_; }
    
};



#endif	/* JOINT_SPACE_PLANNER_H */
