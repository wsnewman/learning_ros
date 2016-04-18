// example_cart_path_planner_main: 
// wsn, April, 2016
// example using cartesian path planner to compute viable horizontal moves for grasp
// choose: yale-gripper z-axis is horizontal; x-axis points up
// try to move horizontally along gripper z axis, preserving orientation
// e.g. to grasp an upright cylinder (such as a cup, bottle, can...)

// uses library of arm-motion planning functions
#include <cartesian_planner/cartesian_planner.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_path_planner_main");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
    //some handy constants...
    Eigen::Matrix3d R_gripper_horiz;
    Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
    Eigen::Vector3d gripper_origin;
    bool found_path = false;

    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
    Eigen::Affine3d a_tool_start, a_tool_end;

    std::vector<Eigen::VectorXd> optimal_path;
    //Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    ROS_INFO("instantiating a cartesian planner object: ");
    CartTrajPlanner cartTrajPlanner;

    ofstream outfile;
    outfile.open("approachable_poses.dat");
    gripper_n_des << 0, 0, 1; //gripper x-axis points straight up;
    double gripper_theta = 0;
    double L_depart = 0.25; // distance to move along gripper z
    double x_des = 0.5;
    double y_des = 0.0;
    double z_des = 0.0;
    gripper_b_des << cos(gripper_theta), sin(gripper_theta), 0;
    gripper_t_des = gripper_b_des.cross(gripper_n_des);
    R_gripper_horiz.col(0) = gripper_n_des;
    R_gripper_horiz.col(1) = gripper_t_des;
    R_gripper_horiz.col(2) = gripper_b_des;

    a_tool_start.linear() = R_gripper_horiz;
    a_tool_end.linear() = R_gripper_horiz;
    for (x_des = 0.2; x_des < 1.5; x_des += 0.1) {
        for (y_des = -1.5; y_des <= 1.5; y_des += 0.1) {
            //for (gripper_theta = 0.0; gripper_theta < 6.28; gripper_theta += 0.2) 
            gripper_theta = M_PI/3.0;
            {
                gripper_b_des << cos(gripper_theta), sin(gripper_theta), 0;
                    gripper_t_des = gripper_b_des.cross(gripper_n_des);
                R_gripper_horiz.col(0) = gripper_n_des;
                R_gripper_horiz.col(1) = gripper_t_des;
                R_gripper_horiz.col(2) = gripper_b_des;
                a_tool_start.linear() = R_gripper_horiz;
                a_tool_end.linear() = R_gripper_horiz;
                
                gripper_origin << x_des, y_des, z_des; //specify gripper pose at grasp position
                a_tool_start.translation() = gripper_origin;
                a_tool_end.translation() = gripper_origin - L_depart*gripper_b_des;

                //bool cartesian_path_planner(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
                found_path = cartTrajPlanner.cartesian_path_planner(a_tool_start, a_tool_end, optimal_path);

                if (found_path) {
                    ROS_INFO("found path; x= %f, y= %f, gripper_theta = %f", x_des, y_des, gripper_theta);
                    outfile<<x_des<<", "<<y_des<<", "<<gripper_theta<<endl;
                            
                } else {
                    ROS_WARN("no path found; x= %f, y= %f, gripper_theta = %f", x_des, y_des, gripper_theta);
                }
            }
        }
    }
    outfile.close();
    return 0;
}
