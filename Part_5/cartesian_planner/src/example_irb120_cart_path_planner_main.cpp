// example_irb120_cart_path_planner_main: 
// wsn, Dec, 2017
// example using cartesian path planner to compute viable horizontal moves for irb120
// choose: tool-flange z-axis pointing down
// try to move horizontally along x axis, preserving orientation

// uses library of arm-motion planning functions
#include <cartesian_planner/irb120_cartesian_planner.h>
#include <irb120_fk_ik/irb120_kinematics.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <fstream>

//specify values here for desired motion
const double z_des = 0.1;
const double y_start = -0.2;
const double y_end = 0.2;
const double x_start = 0.2;
const double x_end =   0.2;

const double x_mid = 0.25;
const double y_mid = 0.0;
const double z_mid = 0.2;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_irb120_cart_path_planner_main");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
    //some handy constants...
    Eigen::Matrix3d R_gripper_horiz, R_gripper_dn, R_rot_z, R_start,R_mid,R_angle_axis;
    Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
    Eigen::Vector3d gripper_n_start, gripper_t_start, gripper_b_start;
    Eigen::Vector3d flange_origin_start,flange_origin_end,flange_origin_mid;
    bool found_path = false;
    
    ofstream outfile; //open a file in which to save the results
    outfile.open("irb120_poses.path");    
    
    flange_origin_start<<x_start,y_start,z_des;
    flange_origin_end<<x_end,y_end,z_des;
    flange_origin_mid<<x_mid,y_mid,z_mid;

    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
    Eigen::Affine3d a_tool_start, a_tool_end, a_tool_mid; //really, these refer to the tool flange

    std::vector<Eigen::VectorXd> optimal_path;
    Eigen::VectorXd qvec;
    //Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    ROS_INFO("instantiating a cartesian planner object: ");
    // this will be an irb10 planner, because we will link with this library
    CartTrajPlanner cartTrajPlanner; //instantiate a cartesian planner object

    R_gripper_dn = cartTrajPlanner.get_R_gripper_down();
    Eigen::Vector3d rotz_n,rotz_t,rotz_b;
    double theta_rot_z = 0.123;
    rotz_n<<cos(theta_rot_z),sin(theta_rot_z),0;
    rotz_t<<-sin(theta_rot_z),cos(theta_rot_z),0;
    rotz_b<<0,0,1;
    R_rot_z.col(0) = rotz_n;
    R_rot_z.col(1) = rotz_t;
    R_rot_z.col(2) = rotz_b;
    //specify start and end poses:
    R_start = R_gripper_dn;
    a_tool_start.linear() = R_start;
    a_tool_start.translation() = flange_origin_start;
    a_tool_end.linear() = R_rot_z*R_gripper_dn;
    a_tool_end.translation() = flange_origin_end;
    //also a mid pose, to test multipoint paths:
    a_tool_mid.translation() = flange_origin_mid;
    Eigen::Vector3d k_rot_axis;
    k_rot_axis<< 0,1,0; // rotate about the y axis
    double theta = 1.0; //by 1 rad
    R_angle_axis = Eigen::AngleAxisd(theta, k_rot_axis);
    R_mid = R_angle_axis*R_start;
    a_tool_mid.linear() = R_mid;
    std::vector<Eigen::Affine3d> a_flange_poses;
    a_flange_poses.push_back(a_tool_start);
     a_flange_poses.push_back(a_tool_mid);
    a_flange_poses.push_back(a_tool_end);   
    std::vector<int> nsteps_vec;
    std::vector<int> nsteps_to_via_pt;
    nsteps_vec.push_back(3);
    nsteps_vec.push_back(3);
    found_path = cartTrajPlanner.multipoint_cartesian_path_planner(a_flange_poses,nsteps_vec, optimal_path,nsteps_to_via_pt);
    if (found_path) {
      ROS_INFO("found multistep path");
    }
    ROS_INFO("joint-space values in complete path: ");
    int n_tot_path_pts = optimal_path.size();
    for (int i=0;i<n_tot_path_pts;i++) {
     cout<<optimal_path[i].transpose()<<endl;
    }
    int n_via_pts = nsteps_to_via_pt.size();
    ROS_INFO("number of steps to each via point: ");
    for (int i=0;i<n_via_pts;i++) {
     cout<<nsteps_to_via_pt[i]<<", ";
    }
    cout<<endl;

   return 0;  //halt here, or remove to do rest of testing    
    //do a Cartesian plan:
    //found_path = cartTrajPlanner.cartesian_path_planner(a_tool_start, a_tool_end, optimal_path);

    //bool cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_tool_start,Eigen::Affine3d a_tool_end, 
    //   int nsteps,   std::vector<Eigen::VectorXd> &optimal_path);
    int nsteps = 4;
    found_path = cartTrajPlanner.cartesian_path_planner_w_rot_interp(a_tool_start, a_tool_end, nsteps, optimal_path);    

    nsteps = optimal_path.size();
    cout<<"there are "<<nsteps<< " layers in the computed optimal path"<<endl;
    if (found_path) {
                    ROS_INFO("found path");
                    for (int istep = 0;istep<nsteps;istep++) {
                        qvec = optimal_path[istep];
                        cout<<"qvec: "<<qvec.transpose()<<endl;
                        for (int ijnt=0;ijnt<NJNTS-1;ijnt++) {
                            //cout<<qvec[ijnt]<<", ";
                            outfile<<qvec[ijnt]<<", ";
                        }
                        //cout<<qvec[NJNTS-1]<<endl;
                        outfile<<qvec[NJNTS-1]<<endl;
                            
                } 
    } else {
                    ROS_WARN("no path found");
                }

    
 
    outfile.close();
    return 0;
}
