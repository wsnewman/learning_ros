// example_arm7dof_cart_path_planner_main2:  reverse path
// wsn, July, 2016
// example using cartesian path planner to compute viable horizontal moves for arm7dof
// choose: tool-flange z-axis pointing up, i.e. flange acts like platter
// try to move horizontally along x axis, preserving orientation
// e.g. to carry a cup, bottle, etc 

// uses library of arm-motion planning functions
#include <cartesian_planner/arm7dof_cartesian_planner.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <fstream>

const double z_des = 1.5;
const double y_des = 0.3;
const double x_start = -1.5;
const double x_end = 1.5;
const int NJNTS=7;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm7dof_cart_path_planner_main");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
    //some handy constants...
    Eigen::Matrix3d R_gripper_horiz, R_gripper_up;
    Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
    Eigen::Vector3d flange_origin_start,flange_origin_end;
    Vectorq7x1 q_start,q_end;
    //q_end<<2.7, 0.472022, 1.00116, 1.15444, -0.39681, -1.43445, -3.59048; 
    q_start<<2.3, -1.34796, -1.00994, 1.15619, -1.57891, -0.971782, -0.376517;
    q_end<<0.2, -0.580717, 2.56523, 0.884733, 0.308473, -1.39618, -2.89911;
    //q_start<<2.7,  -1.20769, -0.760996,  0.759817,  -1.93547, -0.761566, -0.317656;
    bool found_path = false;
    
    ofstream outfile; //open a file in which to save the results
    outfile.open("arm7dof_rvrs.path");    
    
    flange_origin_start<<x_start,y_des,z_des;
    flange_origin_end<<x_end,y_des,z_des;

    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
    Eigen::Affine3d a_tool_start, a_tool_end;

    std::vector<Eigen::VectorXd> optimal_path;
    Eigen::VectorXd qvec;
    //Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    ROS_INFO("instantiating a cartesian planner object: ");
    // this will be an arm7dof planner, because we will link with this library
    CartTrajPlanner cartTrajPlanner; //instantiate a cartesian planner object



    R_gripper_up = cartTrajPlanner.get_R_gripper_up();

    //specify start and end poses:
    a_tool_start.linear() = R_gripper_up;
    a_tool_start.translation() = flange_origin_start;
    a_tool_end.linear() = R_gripper_up;
    a_tool_end.translation() = flange_origin_end;
    
    //do a Cartesian plan:
    found_path = cartTrajPlanner.cartesian_path_planner(q_end, a_tool_start, optimal_path);

    int nsteps = optimal_path.size();
    cout<<"there are "<<nsteps<< " layers in the computed optimal path"<<endl;
    if (found_path) {
                    ROS_INFO("found path");
                    for (int istep = 0;istep<nsteps;istep++) {
                        qvec = optimal_path[istep];
                        cout<<"qvec: "<<qvec.transpose()<<endl;
                        for (int ijnt=0;ijnt<NJNTS-1;ijnt++) {
                            cout<<qvec[ijnt]<<", ";
                            outfile<<qvec[ijnt]<<", ";
                        }
                        cout<<qvec[NJNTS-1]<<endl;
                        outfile<<qvec[NJNTS-1]<<endl;
                            
                } 
    } else {
                    ROS_WARN("no path found");
                }

    
 
    outfile.close();
    return 0;
}
