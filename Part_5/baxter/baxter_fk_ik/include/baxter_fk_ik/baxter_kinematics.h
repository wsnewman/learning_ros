/* 
 * File:   baxter_kinematics.h
 * Author: wsn
 *
 * Created May 26, 2015
 */
//NOTE: IK functions are/ w/rt tool flange frame; 
// calling fnc must first convert desired hand frame to tool-flange frame, e.g.:
//     a_flange_end_ = a_tool_end_*A_tool_wrt_flange_.inverse();


#ifndef BAXTER_KIN_H
#define	BAXTER_KIN_H
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
// a fwd kin solver...
// list DH params here
/* per: https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/5X1-6w-Ja1I
#DH (alpha_i,a_i,d_i,theta_i)
DH=[
[-1.571,0.069,0.2703,s1],
[1.571,0,0,e0],
[-1.571,0.069,0.3644,e1],
[1.571,0,0,w0],
[-1.571,0.01,0.3743,w1],
[1.571,0,0,w2],
[0,0,h,eefa],
]
*/

//frames:  base has x-axis forward, z up, origin at mount plate;
// sequentially, from torso to tip, frames are:
//right_upper_shoulder has z-axis through S0, x-axis at 45-deg (parallel to home pose of outstretched right arm)
//right_lower_shoulder has z-axis through S1, x-axis points forward along humerus axis (outstretched in home pose)
//  origin is "shoulder point"
//right_upper_elbow: z-axis through humerus rotation (E0), and x-axis points DOWN (in home pose);
// origin is offset along (self) z-axis relative to parent frame
//right_lower_elbow: origin at "elbow point" w/ z-axis along elbow bend (E1), w/ x-axis FORWARD (at home)
//right_upper_forearm: origin offset from above (along self z); z-axis along forearm rot (W0)
// right lower_forearm: has origin at "wrist point"; z-axis through wrist bend (W1)
//right_wrist: z-axis points OUT from tool flange--but orgin is offset (btwn wrist pt and flange face); W2 axis
//right_hand: origin on flange face, z-axis points out; x-axis is DOWN (at home)

// at "home" angles (all zeros, w/rt Baxter commands), arms are outstretched w/ first shoulder jnt (S0)
// such that arms are at 45deg from forward (making a 90-deg angle w/rt each other)
// and elbow offsets are BELOW humerus axes

//TOOL TRANSFORM params, right hand
const double Lx_hand = 0; //0 for baxter gripper, vs -0.03 for Yale hand
const double Lz_hand = 0.158; //0.158 for baxter gripper, vs 0.120 for Yale hand;
const double theta_yaw_hand= 0;// 0 for Baxter gripper vs -0.24 for Yale hand

/* need to make this consistent with transform publisher in: cwru_baxter_launch/yale_gripper_xform.launch
 * check w/:
rosrun tf tf_echo right_hand yale_gripper_frame
At time 1440173202.403
- Translation: [-0.030, 0.000, 0.120]
- Rotation: in Quaternion [0.000, 0.000, -0.120, 0.993]
            in RPY (radian) [0.000, 0.000, -0.240]
            in RPY (degree) [0.000, 0.000, -13.751]
 */
// these values for RIGHT ARM
const double DH_a1=0.069;
const double DH_a2=0.0;
const double DH_a3=0.069;
const double DH_a4=0.0;
const double DH_a5=0.01; //forearm is slightly offset from wrist bend
const double DH_a5_approx=0.0; //approx spherical wrist
const double DH_a6=0.0;
const double DH_a7=0.0; // assign final frame at flange face, z-axis pointing out


//robot.DH.d='[0.290 0 0 0.302 0 0.072]';
const double DH_d1 = 0.2703;// ht from mounting plate?
const double DH_d2 = 0.0;
const double DH_d3 = 0.3644; // humerus
const double DH_d4 = 0.0;
const double DH_d5 = 0.3743; //forearm
const double DH_d6 = 0.0;
const double DH_d7 = 0.2295; // DIST FROM WRIST TO FLANGE--or to tool tip

//robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
const double DH_alpha1 = -M_PI/2.0;
const double DH_alpha2 = M_PI/2.0;
const double DH_alpha3 = -M_PI/2.0;
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = -M_PI/2.0;
const double DH_alpha6 = M_PI/2.0;
const double DH_alpha7 = 0.0; // choose last axis coincident w/ last joint axis--i.e., points out of tool flange

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
// need to find offsets to reconcile Baxter home angles w/ D-H home angles
const double DH_q_offset1 = 0.0;
const double DH_q_offset2 = M_PI/2.0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0.0;
const double DH_q_offset5 = 0.0;
const double DH_q_offset6 = 0.0; //M_PI;
const double DH_q_offset7 = 0.0; //M_PI;

const double deg2rad = M_PI/180.0;

//order: S0, S1, E0, E1, W0, W1, W2
// 51, 60, 173.5, 150, 175.25, 120, 175.25
//in simu, found right arm max angle about -1.65 to 1.65 rad...not close to spec of -141 to 51 deg
const double DH_q_max1 = deg2rad*95; //141; //51;
const double DH_q_max2 = deg2rad*60;
const double DH_q_max3 = deg2rad*173.5;
const double DH_q_max4 = deg2rad*150;
const double DH_q_max5 = deg2rad*175.25;
const double DH_q_max6 = deg2rad*120; //
const double DH_q_max7 = deg2rad*175.25;

//-141, -123, -173.5, -3, -175.25, -90, -175.25
const double DH_q_min1 = -deg2rad*95; //51; //141;
const double DH_q_min2 = -deg2rad*123;
const double DH_q_min3 = -deg2rad*173.5; 
const double DH_q_min4 = -deg2rad*3;
const double DH_q_min5 = -deg2rad*175.25;
const double DH_q_min6 = -deg2rad*90; //
const double DH_q_min7 = -deg2rad*175.25;

const double DH_a_params[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
const double DH_a_params_approx[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5_approx,DH_a6,DH_a7};
const double DH_d_params[7] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6,DH_d7};
const double DH_alpha_params[7] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6,DH_alpha7};
const double DH_q_offsets[7] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6,DH_q_offset7};
const double q_lower_limits[7] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6, DH_q_min7};
const double q_upper_limits[7] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6, DH_q_max7};

// REPEAT ALL OF THE ABOVE FOR LEFT ARM


//Baxter defines a "right_arm_mount" frame, fixed w/rt "torso" frame
// right_upper_shoulder frame moves w/rt right_arm_mount frame;
// these frames are parallel, but origins are offset by these amounts:
const double rmount_to_r_lower_forearm_x = 0.055695;
const double rmount_to_r_lower_forearm_y = 0.0;
const double rmount_to_r_lower_forearm_z = 0.011038;

//transform from torso frame to right-arm-mount frame:
// vector to right-arm mount origin, w/rt torso frame:
const double torso_to_rmount_x =  0.024645;
const double torso_to_rmount_y = -0.219645;
const double torso_to_rmount_z = 0.118588;
//axes of arm-mount frame are rotated about z w/rt torso,
// R_rmount_axes_wrt_torso = Rotz(theta_z_arm_mount)
const double theta_z_arm_mount = -M_PI/4;

//some arbitrary, tunable constants:
const double r_goal_max = 0.745; // forbid reach > this, from shoulder to wrist; forces bent elbows and avoids singularities
const double DQS0 = 0.05; // increment dq0 this much is search of indexed null-space solutions
//max number of iters of J_inv for precision soln;
const int MAX_JINV_ITERS = 5;
const double W_ERR_TOL = 0.0001; //100-micron tolerance on precise solution
const double DQ_ITER_MAX = 0.05; // only allow this large of a step per Jacobian iteration

class Baxter_fwd_solver {
public:
    Baxter_fwd_solver(); //constructor
    
    // these functions are for RIGHT ARM ONLY; tool-flange coords, w/rt right-arm mount (default) or w/rt torso (as noted)
    Eigen::Affine3d fwd_kin_flange_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Affine3d fwd_kin_flange_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec);//version w/ spherical-wrist approx
    Eigen::Affine3d fwd_kin_flange_wrt_torso_solve(const Vectorq7x1& q_vec); //rtns pose w/rt torso frame (base frame)

    // these fncs also include transform from flange to tool frame
    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec); // given vector of q angles, compute fwd kin of tool w/rt right-arm mount 
    //in this case, provide the tool transform to be used:
    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve(const Vectorq7x1& q_vec, Eigen::Affine3d A_tool_wrt_flange);

    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec);//version w/ spherical-wrist approx
    //option to provide tool-transform to be used
    Eigen::Affine3d fwd_kin_tool_wrt_r_arm_mount_solve_approx(const Vectorq7x1& q_vec, Eigen::Affine3d A_tool_wrt_flange);//version w/ spherical-wrist approx    
    
    Eigen::Affine3d fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec); //rtns pose w/rt torso frame (base frame)  
    //option to provide the tool transform to use:
    Eigen::Affine3d fwd_kin_tool_wrt_torso_solve(const Vectorq7x1& q_vec, Eigen::Affine3d A_tool_wrt_flange); //rtns pose w/rt torso frame (base frame) 
    
    // these are all w/rt right-arm mount, not torso
    Eigen::Matrix4d get_wrist_frame();
    Eigen::Matrix4d get_shoulder_frame();
    Eigen::Matrix4d get_elbow_frame();
    Eigen::Matrix4d get_flange_frame();  

    Eigen::Matrix4d get_shoulder_frame_approx();
    Eigen::Matrix4d get_elbow_frame_approx();    
    Eigen::Matrix4d get_wrist_frame_approx();   
    Eigen::Matrix4d get_flange_frame_approx();
    Eigen::Matrix3d get_wrist_Jacobian_3x3(double q_s1, double q_humerus, double q_elbow, double q_forearm); //3x3 J for wrist point coords
    Eigen::Vector3d get_wrist_coords_wrt_frame1(const Vectorq7x1& q_vec); //fwd kin from frame 1 to wrist pt
    
    Eigen::MatrixXd compute_Jacobian(const Vectorq7x1& q_vec);
    
    //this fnc casts an affine matrix w/rt torso frame into an affine matrix w/rt right-arm mount frame, so can use fncs above
    Eigen::Affine3d transform_affine_from_torso_frame_to_arm_mount_frame(Eigen::Affine3d pose_wrt_torso);    
    Eigen::Affine3d get_affine_tool_wrt_flange() { return A_tool_wrt_flange_;}
    Eigen::Matrix4d get_A4x4_tool_wrt_flange() { return A4x4_tool_wrt_flange_;}
    Eigen::Matrix4d get_A_torso_to_rarm_mount() { return  A_torso_to_rarm_mount_;}  
    Eigen::Matrix4d get_A4x4_rarm_mount_to_r_lower_forearm() { return  A_rarm_mount_to_r_lower_forearm_;}     
      

    void set_affine_tool_wrt_flange(Eigen::Affine3d A_tool_wrt_flange) { 
        A_tool_wrt_flange_=A_tool_wrt_flange;
        A_tool_wrt_flange_inv_ = A_tool_wrt_flange_.inverse();
    }
    
//private: do not make these private, so IK can access them
    // these member vars are for RIGHT ARM
  
    Eigen::Matrix4d fwd_kin_solve_(const Vectorq7x1& q_vec);
// same as above, but w/ spherical wrist approx
    Eigen::Matrix4d fwd_kin_solve_approx_(const Vectorq7x1& q_vec);  
    
    Eigen::Matrix4d A_mats_[7], A_mat_products_[7]; //, A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::Matrix4d A_mats_approx_[7], A_mat_products_approx_[7];
    Eigen::Matrix4d A_rarm_mount_to_r_lower_forearm_;
    Eigen::Affine3d Affine_rarm_mount_to_r_lower_forearm_;   //initialized, but not used
    Eigen::Matrix4d A_torso_to_rarm_mount_;    
    Eigen::Affine3d Affine_torso_to_rarm_mount_;

    Eigen::Affine3d A_tool_wrt_flange_;
    Eigen::Affine3d A_tool_wrt_flange_inv_;   
    Eigen::Matrix4d A4x4_tool_wrt_flange_; 
    Eigen::MatrixXd Jacobian_; //not used

    // CREATE CORRESPONDING FUNCTIONS FOR LEFT ARM...
};

//BUILD THESE FIRST FOR RIGHT ARM--THEN EMULATE FOR CORRESPONDING LEFT-ARM FNCS
class Baxter_IK_solver:Baxter_fwd_solver  {
public:
    Baxter_IK_solver(); //constructor; 

    Eigen::Affine3d get_flange_frame_from_tool_frame(Eigen::Affine3d tool_frame) { return tool_frame*A_tool_wrt_flange_inv_; }
    // return the number of valid solutions; actual vector of solutions will require an accessor function
    int ik_solve(Eigen::Affine3d const& desired_flange_pose); // 
    void get_solns(std::vector<Vectorq7x1> &q_solns);
    bool fit_joints_to_range(Vectorq7x1 &qvec);
    Eigen::Vector3d wrist_frame0_from_flange_wrt_rarm_mount(Eigen::Affine3d affine_flange_frame);    
    Eigen::Vector3d wrist_frame1_from_flange_wrt_rarm_mount(Eigen::Affine3d,  Vectorq7x1 q_vec); // bad name
    Eigen::Vector3d wrist_pt_from_flange_frame(Eigen::Affine3d affine_flange_frame);    
    //here is identical fnc, w/ better name
    Eigen::Vector3d wrist_pt_wrt_frame1_of_flange_des_and_qs0(Eigen::Affine3d affine_flange_frame, Vectorq7x1 q_vec);
    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
      //given desired flange pose, fill up solns for q1, q2, q3 based on wrist position
    
    // the following functions are approximate--using spherical-wrist approximation
    // i.e., ignoring 1cm offset of wrist point from forearm axis
    
    // given desired hand pose, find all viable IK solns, indexed by q_s0 values, w/ resolution DQS0    
    int ik_solve_approx(Eigen::Affine3d const& desired_flange_pose,std::vector<Vectorq7x1> &q_solns); 
    
    // this version takes arm of desired hand pose w/rt torso frame
    int ik_solve_approx_wrt_torso(Eigen::Affine3d const& desired_flange_pose,std::vector<Vectorq7x1> &q_solns);
    int ik_solve_approx_wrt_torso(Eigen::Affine3d const desired_tool_pose_wrt_torso,
          Eigen::Affine3d A_tool_wrt_flange, std::vector<Vectorq7x1> &q_solns);
    
    int ik_wristpt_solve_approx_wrt_torso(Eigen::Affine3d const& desired_flange_pose_wrt_torso,std::vector<Vectorq7x1> &q_solns); 
    
    int ik_solve_approx_elbow_orbit_from_flange_pose_wrt_torso(Eigen::Affine3d const& desired_flange_pose_wrt_torso,std::vector<std::vector<Eigen::VectorXd> > &path_options);
    int ik_solve_approx_elbow_orbit_plus_qdot_s0_from_flange_pose_wrt_torso(Vectorq7x1 q_start, std::vector<std::vector<Eigen::VectorXd> > &path_options);  
    // in this version, soln ONLY for specified q_s0;  specify q_s0 and desired hand pose, w/rt torso
    // expect from 0 to 4 solutions at given q_s0    
    int ik_solve_approx_wrt_torso_given_qs0(Eigen::Affine3d const& desired_flange_pose_wrt_torso,double q_s0, std::vector<Vectorq7x1> &q_solns);
    int ik_wrist_solve_approx(Eigen::Affine3d const& desired_flange_pose,std::vector<Vectorq7x1> &q_solns_123); // given desired hand pose, find all viable IK solns
    
    //function to find precise values of joint angles q1, q2, q3 to match desired wrist position, implied by desired_flange_pose
    //provide q123_approx; this function will take q_s0 and q_forearm as specified, and q_s1, q_humerus and q_elbow as approximated,
    // and will refine q_s1, q_humerus and q_elbow to attempt a precise fit to desired wrist position;
    // improved soln is returned in q123_precise
    double precise_soln_q123(Eigen::Affine3d const& desired_flange_pose,Vectorq7x1 q123_approx, Vectorq7x1 &q123_precise);
    //fnc to find q_s0_min and q_s0_max given desired hand pose
    double compute_qs0_ctr(Eigen::Affine3d const& desired_flange_pose);
    
// put together [q_s1,q_humerus,q_elbow] solns as fnc (q_s0)
// there will be 0, 1 or 2 solutions;
// pack these into a 7x1 vector--just leave wrist DOFs =0 for now;
// return "true" if at least 1 valid soln within joint ranges    
    bool compute_q123_solns(Eigen::Affine3d const& desired_flange_pose, double q_s0, std::vector<Vectorq7x1> &q_solns);
    //double solve_for_theta2(double q1,Eigen::Vector3d w_des);
    
    bool solve_for_elbow_ang(Eigen::Vector3d w_wrt_1, double  &q_elbow);  
    //given desired wrist point, w/rt frame 1, and given q_elbow, solve for q_humerus
    bool solve_for_humerus_ang(Eigen::Vector3d w_wrt_1,double q_elbow, double q_humerus[2]);
    bool solve_for_s1_ang(Eigen::Vector3d w_wrt_1,double q_elbow, double q_humerus, double &q_s1);   
    //bool solve_for_theta3(Eigen::Vector3d w_wrt_1,double r_goal, double q3_solns[2]); 

    bool solve_spherical_wrist(Vectorq7x1 q_in,Eigen::Matrix3d R_des, std::vector<Vectorq7x1> &q_solns);  
    bool update_spherical_wrist(Vectorq7x1 q_in,Eigen::Matrix3d R_des, Vectorq7x1 &q_precise);
    bool improve_7dof_soln(Eigen::Affine3d const& desired_flange_pose_wrt_arm_mount, Vectorq7x1 q_in, Vectorq7x1 &q_7dof_precise);
    bool improve_7dof_soln_wrt_torso(Eigen::Affine3d const& desired_flange_pose_wrt_torso, Vectorq7x1 q_in, Vectorq7x1 &q_7dof_precise);

private:
    bool fit_q_to_range(double q_min, double q_max, double &q);    
    std::vector<Vectorq7x1> q7dof_solns;
    std::vector<Vectorq7x1> q_solns_fit;
    //Eigen::Matrix4d A_mats[7], A_mat_products[7], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus_;
    double L_forearm_;
    //double phi_elbow_;
    double phi_shoulder_;
  
    //Eigen::MatrixXd Jacobian;
};

#endif	/* BAXTER_KIN_H */

