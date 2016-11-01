// example_ur10_cart_path_planner_main: 
// wsn, Nov, 2016
// example using cartesian path planner to compute viable horizontal moves for UR10
// choose: tool-flange z-axis pointing down
// try to move horizontally along x axis, preserving orientation

// uses library of arm-motion planning functions
#include <cartesian_planner/ur10_cartesian_planner.h>
#include <ur_fk_ik/ur_kin.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <fstream>

const double z_des = 0.1;
const double y_des = 0.3;
const double x_start = 0.8;
const double x_end = -0.8;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_ur10_cart_path_planner_main");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
    //some handy constants...
    Eigen::Matrix3d R_gripper_horiz, R_gripper_dn;
    Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
    Eigen::Vector3d flange_origin_start,flange_origin_end;
    bool found_path = false;
    
    ofstream outfile; //open a file in which to save the results
    outfile.open("ur10_poses.path");    
    
    flange_origin_start<<x_start,y_des,z_des;
    flange_origin_end<<x_end,y_des,z_des;

    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
    Eigen::Affine3d a_tool_start, a_tool_end;

    std::vector<Eigen::VectorXd> optimal_path;
    Eigen::VectorXd qvec;
    //Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    ROS_INFO("instantiating a cartesian planner object: ");
    // this will be a ur10 planner, because we will link with this library
    CartTrajPlanner cartTrajPlanner; //instantiate a cartesian planner object



    R_gripper_dn = cartTrajPlanner.get_R_gripper_down();

    //specify start and end poses:
    a_tool_start.linear() = R_gripper_dn;
    a_tool_start.translation() = flange_origin_start;
    a_tool_end.linear() = R_gripper_dn;
    a_tool_end.translation() = flange_origin_end;
    
    //do a Cartesian plan:
    found_path = cartTrajPlanner.cartesian_path_planner(a_tool_start, a_tool_end, optimal_path);
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
