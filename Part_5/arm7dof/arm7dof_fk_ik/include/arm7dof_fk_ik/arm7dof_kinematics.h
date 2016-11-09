/* 
 * File:   arm7dof_kinematics.h
 * Author: wsn
 *
 * Created July, 2016
 */
//NOTE: IK functions are/ w/rt tool flange frame; 
// calling fnc must first convert desired hand frame to tool-flange frame, e.g.:
//     a_flange_end_ = a_tool_end_*A_tool_wrt_flange_.inverse();


#ifndef ARM7DOF_KIN_H
#define	ARM7DOF_KIN_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>


typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;


//TOOL TRANSFORM params (not used yet)
const double Lx_hand = 0;
const double Lz_hand = 0;
const double theta_yaw_hand= 0;

const double DH_a1=0.0;
const double DH_a2=0.0;
const double DH_a3=0.0;
const double DH_a4=0.0;
const double DH_a5=0.0; 
//const double DH_a5_approx=0.0; //approx spherical wrist
const double DH_a6=0.0;
const double DH_a7=0.0; // assign final frame at flange face, z-axis pointing out

//Need to reconcile these DH values: 
const double DH_d1 = 0.35;
const double DH_d2 = -0.25;  //non-obvious sign
const double DH_d3 = 1; 
const double DH_d4 = 0.2; //non-obvious sign
const double DH_d5 = 1; 
const double DH_d6 = 0.0;
const double DH_d7 = 0.2; 

//NOTE: reconcile the above values w/ defs in arm7dof_model.xacro:

//<xacro:property name="DH_d1" value="0.35" />
//<xacro:property name="DH_d2" value="0.25" />
//<xacro:property name="DH_d3" value="1" />
//<xacro:property name="DH_d4" value="-0.2" />
//<xacro:property name="DH_d5" value="1" />
//<xacro:property name="DH_d6" value="0.0" />
//<xacro:property name="DH_d7" value="0.2" />

const double DH_alpha1 = M_PI/2.0;
const double DH_alpha2 = -M_PI/2.0;
const double DH_alpha3 = M_PI/2.0;
const double DH_alpha4 = -M_PI/2.0;
const double DH_alpha5 = M_PI/2.0;
const double DH_alpha6 = -M_PI/2.0;
const double DH_alpha7 = 0.0; // choose last axis coincident w/ last joint axis--i.e., points out of tool flange

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
// offsets to reconcile model home angles w/ D-H home angles
const double DH_q_offset1 = 0.0;
const double DH_q_offset2 = 0.0; 
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0.0;
const double DH_q_offset5 = 0.0;
const double DH_q_offset6 = 0.0; 
const double DH_q_offset7 = 0.0; 

const double deg2rad = M_PI/180.0;
//reconcile w/ URDF:
//jnt0:     <limit lower="-3" upper="3" effort="1300" velocity="0.2"/>
//jnt1:     <limit lower="-3" upper="3" effort="1300" velocity="0.2"/>
//jnt2:     <limit lower="-3.1" upper="2.8" effort="500" velocity="0.3"/>
//jnt3:     <limit lower="-3.1" upper="2.7" effort="500" velocity="0.3"/>
//jnt4:     <limit lower="-4." upper="1.5" effort="100" velocity="0.400"/>
//jnt5:     <limit lower="-3" upper="3" effort="100" velocity="0.5"/>
//jnt6:     <limit lower="-4.5" upper="1.2" effort="70" velocity="0.4"/>

const double DH_q_max1 = 3; //deg2rad*160; //95; //141; //51;
const double DH_q_max2 = 3; //deg2rad*160; //60;
const double DH_q_max3 = 2.8; //deg2rad*147.5; //173.5;
const double DH_q_max4 = 2.7; //deg2rad*150; //150;
const double DH_q_max5 = 1.5; //deg2rad*70; //175.25;
const double DH_q_max6 = 3; //deg2rad*160; //120; //
const double DH_q_max7 = 1.2; //deg2rad*67; //175.25;


const double DH_q_min1 = -3; //-deg2rad*160; //51; //141;
const double DH_q_min2 = -3; //-deg2rad*160;
const double DH_q_min3 = -3.1; //-deg2rad*172.5; 
const double DH_q_min4 = -3.1; //-deg2rad*170;
const double DH_q_min5 = -4; //-deg2rad*250;
const double DH_q_min6 = -3; //-deg2rad*160; //
const double DH_q_min7 = -4.5; //-deg2rad*247;

const double DH_a_params[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
//const double DH_a_params_approx[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5_approx,DH_a6,DH_a7};
const double DH_d_params[7] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6,DH_d7};
const double DH_alpha_params[7] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6,DH_alpha7};
const double DH_q_offsets[7] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6,DH_q_offset7};
const double q_lower_limits[7] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6, DH_q_min7};
const double q_upper_limits[7] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6, DH_q_max7};



//some arbitrary, tunable constants:
//const double r_goal_max = 0.745; // forbid reach > this, from shoulder to wrist; forces bent elbows and avoids singularities
const double DQ_YAW = 0.1; // increment dq0 this much is search of indexed null-space solutions
//max number of iters of J_inv for precision soln;
//const int MAX_JINV_ITERS = 5;
const double W_ERR_TOL = 0.0001; //100-micron tolerance on precise solution
//const double DQ_ITER_MAX = 0.05; // only allow this large of a step per Jacobian iteration
//const double g_qdot_max_vec[] = {2.16, 2.16, 3.15, 3.2, 3.2, 3.2};
const double g_qdot_max_vec[] = {0.2, 0.2, 0.3, 0.3, 0.4, 0.5, 0.4}; //values per URDF 

class Arm7dof_fwd_solver {
public:
    Arm7dof_fwd_solver(); //constructor
    

        //fwd kin fncs: tool flange w/rt base (link1)
    Eigen::Affine3d fwd_kin_flange_wrt_base_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin

    // these fncs also include transform from flange to tool frame
    Eigen::Affine3d fwd_kin_tool_wrt_base_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin of tool w/rt right-arm mount 
    Eigen::MatrixXd Jacobian(Eigen::VectorXd q_vec);
    //get coords of wrist point w/rt frame0;
    // provide q_vec:
    Eigen::Vector3d get_wrist_point(const Vectorq7x1& q_vec);  
    //or use this, if fwd kin has already been computed   
    Eigen::Vector3d get_wrist_point(); 
    
    //get coords of wrist pt w/rt frame1:
    Eigen::Vector3d get_wrist_coords_wrt_frame1(const Vectorq7x1& q_vec); 
    //or use this version if fwd kin has already been computed
    Eigen::Vector3d get_wrist_coords_wrt_frame1(); 
        
    // these are all w/rt base link
    Eigen::Matrix4d get_frame(int i) {return A_mat_products_[i];};     
    Eigen::Matrix4d get_frame0() {return A_mat_products_[0];};    
    Eigen::Matrix4d get_frame1() {return A_mat_products_[1];};
    Eigen::Matrix4d get_frame2() {return A_mat_products_[2];};
    Eigen::Matrix4d get_frame3() {return A_mat_products_[3];};
    Eigen::Matrix4d get_frame4() {return A_mat_products_[4];};  
    Eigen::Matrix4d get_frame5() {return A_mat_products_[5];};      
    Eigen::Matrix4d get_frame6() {return A_mat_products_[6];}; 
       
    Eigen::Affine3d get_affine_tool_wrt_flange() { return A_tool_wrt_flange_;}
    
        //inner fwd-kin fnc: computes tool-flange frame w/rt base frame
        //return soln out to tool flange; would still need to account for tool transform for gripper    
    Eigen::Matrix4d fwd_kin_solve_(const Vectorq7x1& q_vec);
    Eigen::Matrix4d fwd_kin_solve_(Eigen::VectorXd q_vec);
    
    Eigen::Matrix4d A_mats_[7], A_mat_products_[7]; 


    Eigen::Affine3d A_tool_wrt_flange_;
    Eigen::Affine3d A_tool_wrt_flange_inv_;    
    Eigen::MatrixXd Jacobian_; 

};


class Arm7dof_IK_solver:Arm7dof_fwd_solver  {
public:
    Arm7dof_IK_solver(); //constructor; 

//compute elbow solutions given wrist pt w/rt base and given q_yaw (q0);
//return "true" if at least 1 viable soln
//solutions are put in vector q_elbow_solns    
  bool solve_for_elbow_ang(Eigen::Vector3d w_wrt_0, double q_yaw, std::vector<double> &q_elbow_solns);
Eigen::Vector3d get_frame2_origin_of_shoulder_yaw(double q_yaw);
//solve the eqn r = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid
bool solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns);
//given wrist position w/rt frame 0, and given q_yaw and q_elbow, solve for q_humerus options
bool solve_for_humerus_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_elbow, std::vector<double> &q_humerus_solns);

bool fit_q_to_range(double q_min, double q_max, double &q);

bool solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus, double q_elbow, std::vector<double> &q_shoulder_solns);

bool ik_wrist_solns_of_q0(Eigen::Vector3d wrist_pt, double q_yaw, std::vector<Eigen::VectorXd> &q_solns);

Eigen::Vector3d wrist_pnt_from_flange_frame(Eigen::Affine3d affine_flange_frame);

bool solve_spherical_wrist(Vectorq7x1 q_in,Eigen::Matrix3d R_des, std::vector<Vectorq7x1> &q_solns);

int ik_solve_given_qs0(Eigen::Affine3d const& desired_flange_pose_wrt_base,double q_yaw, std::vector<Vectorq7x1> &q_solns);

int ik_solns_sampled_qs0(Eigen::Affine3d const& desired_flange_pose_wrt_base,std::vector<Vectorq7x1> &q_solns);

void test_IK_solns(std::vector<Vectorq7x1> &q_solns);

};

#endif	

