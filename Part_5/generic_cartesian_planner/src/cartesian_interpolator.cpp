// generic cartesian interpolator
// wsn, Dec, 2017 

//#include <generic_cartesian_planner/cartesian_interpolator.h>
#include <cartesian_interpolator/cartesian_interpolator.h>
using namespace std;


//constructor:
CartesianInterpolator::CartesianInterpolator() 
{
   //define a fixed orientation: tool flange pointing down, with x-axis forward
    b_des_ << 0, 0, -1;
    n_des_ << 1, 0, 0;
    t_des_ = b_des_.cross(n_des_);
    
    R_gripper_down_.col(0) = n_des_;
    R_gripper_down_.col(1) = t_des_;
    R_gripper_down_.col(2) = b_des_;
    
   //define a fixed orientation: tool flange pointing up, with x-axis forward
    b_des_up_ << 0, 0, 1;
    n_des_up_ << 1, 0, 0;
    t_des_up_ = b_des_up_.cross(n_des_up_);
    
    R_gripper_up_.col(0) = n_des_up_;
    R_gripper_up_.col(1) = t_des_up_;
    R_gripper_up_.col(2) = b_des_up_;    
    
    // define a fixed orientation corresponding to horizontal tool normal, 
    //  vector between fingers also horizontal
    // right-hand gripper approach direction along y axis
    // useful, e.g. for picking up a bottle to be poured
    tool_n_des_horiz_<<1,0,0;
    tool_b_des_horiz_<<0,1,0;
    tool_t_des_horiz_ = tool_b_des_horiz_.cross(tool_n_des_horiz_);
    R_gripper_horiz_.col(0) = tool_n_des_horiz_;
    R_gripper_horiz_.col(1) = tool_t_des_horiz_;
    R_gripper_horiz_.col(2) = tool_b_des_horiz_;   
    
   
    //default path sampling values:
    //cartesian_path_sample_spacing_ = CARTESIAN_PATH_SAMPLE_SPACING;
    //cartesian_path_fine_sample_spacing_ = CARTESIAN_PATH_FINE_SAMPLE_SPACING;
}


//here's a fnc that does linear interpolation of both translation and rotation
//interpolation of orientation is defined as follows:
// start at R_start, end at R_end--> do a rotation R_change s.t. R_end = R_change*R_start
//  and thus R_change = R_end*R_start_inv
//  define R_change in terms of Rot(k_vec,theta)
//  then sample the path at theta(s), defining goal orientations as Rot(k_vec,theta(s))*R_start
// specify the desired number of steps (pts-1) in the argument
//
// resulting cartesian-space sampling is contained in cart_space_samples
// 

            
bool CartesianInterpolator::cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::Affine3d> &cartesian_affine_samples) {
    
    if (nsteps<1) {
        ROS_WARN("cannot interpolate with nsteps = %d",nsteps);
        return false;       
    }
    
    Eigen::Affine3d a_flange_des;

     Eigen::Matrix3d R_start,R_end,R_change,R_change_interp,R_interp;
     R_start = a_flange_start.linear();
     R_end = a_flange_end.linear();
     //R_end = R_change*R_start
     R_change = R_end*R_start.transpose();
     Eigen::AngleAxisd angleAxis(R_change);  //convert rotation matrix to angle/axis

    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples.clear();
    cartesian_affine_samples.push_back(a_flange_start);


     Eigen::Vector3d k_rot_axis;
     Eigen::Vector3d dp_vec,O_interp,O_start,O_end;
     O_start = a_flange_start.translation();
     O_end = a_flange_end.translation();
     dp_vec = (O_end-O_start)/nsteps;
     //cout<<"O_start = "<<O_start.transpose()<<endl;
     //cout<<"O_end = "<<O_end.transpose()<<endl;
     //cout<<"dp_vec = "<<dp_vec.transpose()<<endl;
     
     
     //interpolate with angle theta_interp about k_rot_axis
     double angle_axis_theta,theta_interp,dtheta;
     angle_axis_theta = angleAxis.angle();
     k_rot_axis = angleAxis.axis();
     //cout<<"k_rot_axis = "<<k_rot_axis.transpose()<<endl;
     //cout<<"angle_axis_theta = "<<angle_axis_theta<<endl;
     //cout<<"R_start: "<<endl;
     //cout<<R_start<<endl;
     //cout<<"R_end: "<<endl;
     //cout<<R_end<<endl;
     theta_interp = 0.0;
     dtheta = angle_axis_theta/nsteps;
     cout<<"R_interp at start: "<<endl;
     cout<<R_start<<endl;
     for (int i=0;i<nsteps;i++) {
        theta_interp = (i+1)*dtheta;        
        R_change_interp = Eigen::AngleAxisd(theta_interp, k_rot_axis);
        R_interp= R_change_interp*R_start;
        //cout<<"at i= "<<i<<", theta = "<<theta_interp<<" R_interp = "<<endl;   
        //cout<<R_interp<<endl;
        O_interp = O_start+(i+1)*dp_vec;
        //cout<<"O_interp = "<<O_interp.transpose()<<endl<<endl;
        a_flange_des.linear() = R_interp;
        a_flange_des.translation() = O_interp;
        cartesian_affine_samples.push_back(a_flange_des);
     }
     //now have a vector of Cartesian samples in cartesian_affine_samples
     return true; //success
}

//this version takes a sequence of Cartesian-space via points and interpolates them, storing
// the result in cartesian_affine_samples.  Additionally, the vector nsteps_to_via_pt is
// populated with the number of steps to each via point
bool CartesianInterpolator::multipoint_cartesian_path_planner(std::vector<Eigen::Affine3d> a_flange_poses,std::vector<int> nsteps_vec, 
std::vector<Eigen::Affine3d> &cartesian_affine_samples, std::vector<int> &nsteps_to_via_pt) {
  Eigen::Affine3d a_start,a_end;
  int nsamp_pts;
  int n_via_pts = a_flange_poses.size();
  if (n_via_pts<2) {
    ROS_WARN("n_via_pts = %d; too few poses for path planning",n_via_pts);
    return false;
  }
  a_start = a_flange_poses[0];
  a_end = a_flange_poses[1];
  nsamp_pts = nsteps_vec[0];
  bool good_path;
  std::vector<Eigen::Affine3d> partial_affine_samples;
  //cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
  //      int nsteps,  std::vector<Eigen::Affine3d> &cartesian_affine_samples)
  good_path = cartesian_path_planner_w_rot_interp(a_start, a_end, nsamp_pts,  partial_affine_samples);

  //append this Cartesian sampled path
  cartesian_affine_samples.clear();
  int npts = partial_affine_samples.size();
  nsteps_to_via_pt.push_back(npts-1);  //there are this many STEPS to the first via point
  for (int i=0;i<npts;i++) {
   cartesian_affine_samples.push_back(partial_affine_samples[i]);
  }
  //do the rest of the via points:
  for (int ivia = 2;ivia< n_via_pts;ivia++) {
     //ROS_INFO("via point %d",ivia);
     a_start = a_end;
     a_end = a_flange_poses[ivia];
     nsamp_pts = nsteps_vec[ivia-1];
     good_path = cartesian_path_planner_w_rot_interp(a_start, a_end, nsamp_pts,  partial_affine_samples);
     //append this sampled path segment
     npts = partial_affine_samples.size();
    for (int i=1;i<npts;i++) { //skip first point,since this is a repeat of prior end point
       cartesian_affine_samples.push_back(partial_affine_samples[i]);
    }
    npts = cartesian_affine_samples.size();
    nsteps_to_via_pt.push_back(npts-1); //number of STEPS to the current via point, from start of polyline

  }

  return true;
}
