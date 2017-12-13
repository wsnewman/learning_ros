// cartesian planner customized for irb120 robot
// cart_path_planner_lib: 
// wsn, Dec, 2017
// a library of arm-motion planning functions
// uses package joint_space_planner to find a good joint-space path among options
// from IK solutions
#include <cartesian_planner/irb120_cartesian_planner.h>
using namespace std;

//constructor:
CartTrajPlanner::CartTrajPlanner() // optionally w/ args, e.g. 
//: as_(nh_, "CartTrajPlanner", boost::bind(&cartTrajActionServer::executeCB, this, _1), false)
{
    ROS_INFO("in constructor of CartTrajPlanner...");
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
    
    jspace_planner_weights_.resize(NJNTS);
    jspace_planner_weights_[0] = 5; //turret
    //penalize shoulder lift more:
    jspace_planner_weights_[1] = 5; //shoulder
    jspace_planner_weights_[2] = 3; //elbow
    //penalize wrist less
    jspace_planner_weights_[3] = 0.5;
    jspace_planner_weights_[4] = 0.2;
    jspace_planner_weights_[5] = 0.2;     
}

bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    return cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_SAMPLE_SPACING);
}

//specify start and end poses w/rt base.  Only orientation of end pose will be considered; orientation of start pose is ignored

bool CartTrajPlanner::cartesian_path_planner(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_des.linear() = R_des;
 

    //a_flange_start = fwdSolver_.fwd_kin_solve(q_start);
    //cout << "fwd kin from q_start: " << a_flange_start.translation().transpose() << endl;
    //cout << "fwd kin from q_start R: " << endl;
    //cout << a_flange_start.linear() << endl;
    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples_.clear();
    a_flange_des = a_flange_end;
    a_flange_start.linear() = R_des; // no interpolation of orientation; set goal orientation immediately   
    a_flange_des.linear() = R_des; //expected behavior: will try to achieve orientation first, before translating
            
    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end-p_start;
    double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm()/dp_scalar);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    nsteps++; //account for pose at step 0


    std::vector<Eigen::VectorXd> q_solns;
    p_des = p_start;
    cartesian_affine_samples_.push_back(a_flange_start);
    for (int istep=0;istep<nsteps;istep++) 
    {
            a_flange_des.translation() = p_des;
            cartesian_affine_samples_.push_back(a_flange_des);
            cout<<"trying: "<<p_des.transpose()<<endl;
            //int ik_solve(Eigen::Affine3d const& desired_hand_pose,vector<Eigen::VectorXd> &q_ik_solns);
            nsolns = ik_solver_.ik_solve(a_flange_des, q_solns);
            std::cout<<"cartesian step "<<istep<<" has = "<<nsolns<<" IK solns"<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    //node = q_solns[isoln];
                    single_layer_nodes[isoln] = q_solns[isoln]; //node;
                    //single_layer_nodes = q_solns; 
                }

                path_options.push_back(single_layer_nodes);
            }
            p_des += dp_vec;
    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}

void CartTrajPlanner::test_IK_solns(std::vector<Eigen::VectorXd> &q_solns) {
    cout<<"testing IK solns: "<<endl;
    int nsolns = q_solns.size();
    Eigen::Affine3d affine_fk;
    Eigen::Vector3d origin_fk;
    for (int isoln = 0; isoln < nsolns; isoln++) {
        affine_fk = fwd_solver_.fwd_kin_solve(q_solns[isoln]);
        origin_fk=affine_fk.translation();
        cout<<origin_fk.transpose()<<endl;
    }
}

//bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start,Eigen::Affine3d a_tool_end, 
//        std::vector<Eigen::VectorXd> &optimal_path) {
//    return cartesian_path_planner(q_start,a_tool_end,optimal_path,CARTESIAN_PATH_SAMPLE_SPACING);
//}

//here's a variation that does linear interpolation of both translation and rotation
//interpolation of orientation is defined as follows:
// start at R_start, end at R_end--> do a rotation R_change s.t. R_end = R_change*R_start
//  and thus R_change = R_end*R_start_inv
//  define R_change in terms of Rot(k_vec,theta)
//  then sample the path at theta(s), defining goal orientations as Rot(k_vec,theta(s))*R_start
// 

bool CartTrajPlanner::cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::VectorXd> &optimal_path) {

    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des;

     Eigen::Matrix3d R_start,R_end,R_change,R_change_interp,R_interp;
     R_start = a_flange_start.linear();
     R_end = a_flange_end.linear();
     //R_end = R_change*R_start
     R_change = R_end*R_start.transpose();
     Eigen::AngleAxisd angleAxis(R_change);  //convert rotation matrix to angle/axis
     
     optimal_path.clear();

    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples_.clear();
    a_flange_des = a_flange_end;
    //a_flange_start.linear() = R_des; // no interpolation of orientation; set goal orientation immediately   
    //a_flange_des.linear() = R_des; //expected behavior: will try to achieve orientation first, before translating
            
    int nsolns;
    bool reachable_proposition;
 
    //Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    //p_start = a_flange_start.translation();
    //p_end = a_flange_end.translation();
    //del_p = p_end-p_start;
    //double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    //nsteps = round(del_p.norm()/dp_scalar);
    //if (nsteps<1) nsteps=1;
    //dp_vec = del_p/nsteps;
    //nsteps++; //account for pose at step 0


    std::vector<Eigen::VectorXd> q_solns;
    //p_des = p_start;
    cartesian_affine_samples_.push_back(a_flange_start);

     //to interpolate to angle theta_interp:
     Eigen::Vector3d k_rot_axis;
     Eigen::Vector3d dp_vec,O_interp,O_start,O_end;
     O_start = a_flange_start.translation();
     O_end = a_flange_end.translation();
     dp_vec = (O_end-O_start)/nsteps;
     cout<<"O_start = "<<O_start.transpose()<<endl;
     cout<<"O_end = "<<O_end.transpose()<<endl;
     cout<<"dp_vec = "<<dp_vec.transpose()<<endl;

     double angle_axis_theta,theta_interp,dtheta;
     angle_axis_theta = angleAxis.angle();
     k_rot_axis = angleAxis.axis();
     cout<<"k_rot_axis = "<<k_rot_axis.transpose()<<endl;
     cout<<"angle_axis_theta = "<<angle_axis_theta<<endl;
     cout<<"R_start: "<<endl;
     cout<<R_start<<endl;
     cout<<"R_end: "<<endl;
     cout<<R_end<<endl;
     theta_interp = 0.0;
     dtheta = angle_axis_theta/nsteps;
     cout<<"R_interp at start: "<<endl;
     cout<<R_start<<endl;
     for (int i=0;i<nsteps;i++) {
        theta_interp = (i+1)*dtheta;        
        R_change_interp = Eigen::AngleAxisd(theta_interp, k_rot_axis);
        R_interp= R_change_interp*R_start;
        cout<<"at i= "<<i<<", theta = "<<theta_interp<<" R_interp = "<<endl;   
        cout<<R_interp<<endl;
        O_interp = O_start+(i+1)*dp_vec;
        cout<<"O_interp = "<<O_interp.transpose()<<endl<<endl;
        a_flange_des.linear() = R_interp;
        a_flange_des.translation() = O_interp;
        cartesian_affine_samples_.push_back(a_flange_des);
     }
     //now have a vector of Cartesian samples in cartesian_affine_samples_
     //for each such sample, compute IK options:
    int nsamps = cartesian_affine_samples_.size();
    //int nsolns;
    for (int istep=0;istep<nsamps;istep++) 
    {
            a_flange_des = cartesian_affine_samples_[istep];
            nsolns = ik_solver_.ik_solve(a_flange_des, q_solns);
            std::cout<<"cartesian step "<<istep<<" has = "<<nsolns<<" IK solns"<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    //node = q_solns[isoln];
                    single_layer_nodes[isoln] = q_solns[isoln]; //node;
                    //single_layer_nodes = q_solns; 
                }

                path_options.push_back(single_layer_nodes);
            }
         else {
            ROS_WARN("no valid IK soln...");
            return false;
          }

    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}

bool CartTrajPlanner::multipoint_cartesian_path_planner(std::vector<Eigen::Affine3d> a_flange_poses,std::vector<int> nsteps_vec, 
std::vector<Eigen::VectorXd> &optimal_path, std::vector<int> &nsteps_to_via_pt) {
  Eigen::Affine3d a_start,a_end;
  int nsamp_pts;
  std::vector<Eigen::VectorXd> partial_path;
  int n_via_pts;
  n_via_pts = a_flange_poses.size();
  if (n_via_pts<2) {
    ROS_WARN("too few poses for path planning");
    return false;
  }
  a_start = a_flange_poses[0];
  a_end = a_flange_poses[1];
  nsamp_pts = nsteps_vec[0];
  bool good_path;
  good_path = cartesian_path_planner_w_rot_interp(a_start, a_end, nsamp_pts,  partial_path);
  if (!good_path) {
    ROS_WARN("IK failed for first segment of path");
    return false;
  }
  //else, partial_path is good, so copy it to optimal_path
  optimal_path.clear();
  int npts = partial_path.size();
  nsteps_to_via_pt.push_back(npts-1);  //there are this many STEPS to the first via point
  for (int i=0;i<npts;i++) {
   optimal_path.push_back(partial_path[i]);
  }
  //do the rest of the via points:
  for (int ivia = 2;ivia< n_via_pts;ivia++) {
     ROS_INFO("via point %d",ivia);
     a_start = a_end;
     a_end = a_flange_poses[ivia];
     nsamp_pts = nsteps_vec[ivia-1];
     good_path = cartesian_path_planner_w_rot_interp(a_start, a_end, nsamp_pts,  partial_path);
     if (!good_path) {
       ROS_WARN("IK failed for first segment of path");
       return false;
     }
     //else, partial_path is good, so copy it to optimal_path
     npts = partial_path.size();
    for (int i=1;i<npts;i++) { //skip first point,since this is a repeat of prior end point
       optimal_path.push_back(partial_path[i]);
    }
    npts = optimal_path.size();
    nsteps_to_via_pt.push_back(npts-1); //number of STEPS to the current via point, from start of polyline

  }
  
  ROS_INFO("path plan through %d via points completed",n_via_pts);

  return true;
}

bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start,Eigen::Affine3d a_tool_end, 
          std::vector<Eigen::VectorXd> &optimal_path, double dp_scalar) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_tool_des,a_tool_start;
    Eigen::Matrix3d R_des = a_tool_end.linear();
    //affine_flange = arm7dof_fwd_solver.fwd_kin_solve(g_q_vec);
    a_tool_start = fwd_solver_.fwd_kin_solve(q_start);
    cout<<"fwd kin from q_start: "<<a_tool_start.translation().transpose()<<endl;
    cout<<"fwd kin from q_start R: "<<endl;
    cout<<a_tool_start.linear()<<endl;
    
    a_tool_start.linear() = R_des; // no interpolation of orientation; set goal orientation immediately   
    a_tool_des.linear() = R_des; //expected behavior: will try to achieve orientation first, before translating

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    p_start = a_tool_start.translation();
    p_end = a_tool_end.translation();
    del_p = p_end-p_start;
    cout<<"p_start: "<<p_start.transpose()<<endl;
    cout<<"p_end: "<<p_end.transpose()<<endl;
    cout<<"del_p: "<<del_p.transpose()<<endl;
    //double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm()/dp_scalar);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    cout<<"dp_vec for nsteps = "<<nsteps<<" is: "<<dp_vec.transpose()<<endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);   

    std::vector<Eigen::VectorXd> q_solns;
    p_des = p_start;
    int ans;
    for (int istep=1;istep<nsteps;istep++) 
    {
            p_des += dp_vec;
            a_tool_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            //note: ik_solver assumes a_tool_des references the DH flange frame w/ z-axis pointing out from flange
            nsolns = ik_solver_.ik_solve(a_tool_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            //DEBUG:
            //test_IK_solns(q_solns);
            //cout<<"enter 1: ";
            //cin>>ans;
            
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                   // node = q_solns[isoln];
                    single_layer_nodes[isoln] = q_solns[isoln]; //node;
                }

                path_options.push_back(single_layer_nodes);
            }
            else {
                return false;
            }

    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    test_IK_solns(optimal_path);
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}

// alt version: specify start as a q_vec, and desired delta-p Cartesian motion while holding R fixed
bool CartTrajPlanner::cartesian_path_planner_delta_p(Eigen::VectorXd q_start, Eigen::Vector3d delta_p, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_tool_des,a_tool_start,a_tool_end;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;

    a_tool_start = fwd_solver_.fwd_kin_solve(q_start);
    Eigen::Matrix3d R_des = a_tool_start.linear();    
    p_des = a_tool_start.translation();
    cout<<"fwd kin from q_start p: "<<p_des.transpose()<<endl;
    cout<<"fwd kin from q_start R: "<<endl;
    cout<<a_tool_start.linear()<<endl;
    
    //a_tool_start.linear() = R_des; // override the orientation component--require point down    
    //construct a goal pose, based on start pose:
    a_tool_end.linear() = R_des;
    a_tool_des.linear() = R_des; // variable used to hold steps of a_des...always with same R
    a_tool_end.translation() = p_des+delta_p;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;

    p_start = a_tool_start.translation();
    p_end = a_tool_end.translation();
    del_p = p_end-p_start; // SHOULD be same as delta_p input
    cout<<"p_start: "<<p_start.transpose()<<endl;
    cout<<"p_end: "<<p_end.transpose()<<endl;
    cout<<"del_p: "<<del_p.transpose()<<endl;
    double dp_scalar = 0.05;
    nsteps = round(del_p.norm()/dp_scalar);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    cout<<"dp_vec for nsteps = "<<nsteps<<" is: "<<dp_vec.transpose()<<endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    //node = q_start;
    single_layer_nodes.push_back(q_start);
    path_options.push_back(single_layer_nodes);   

    std::vector<Eigen::VectorXd> q_solns;
    p_des = p_start;

    for (int istep=1;istep<nsteps;istep++) 
    {
            p_des += dp_vec;
            a_tool_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = ik_solver_.ik_solve(a_tool_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    //node = q_solns[isoln];
                    single_layer_nodes[isoln] = q_solns[isoln];//node;
                }

                path_options.push_back(single_layer_nodes);
            }
            else {
                return false;
            }

    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(NJNTS);
    for (int i = 0; i < NJNTS; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}


   bool CartTrajPlanner::jspace_trivial_path_planner(Eigen::VectorXd q_start,Eigen::VectorXd q_end,std::vector<Eigen::VectorXd> &optimal_path) {
       Eigen::VectorXd qx_start(NJNTS),qx_end(NJNTS);// need to convert to this type

       cout<<"jspace_trivial_path_planner: "<<endl;
       cout<<"q_start: "<<q_start.transpose()<<endl;
       cout<<"q_end: "<<q_end.transpose()<<endl;
       optimal_path.clear();
       optimal_path.push_back(q_start);
       optimal_path.push_back(q_end);
       return true;
   }
   
   //this version uses a finer Cartesian sampling dp than the default

bool CartTrajPlanner::fine_cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    //do trajectory planning w/ fine samples along Cartesian direction, but approximate IK solns:
    bool valid = cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_FINE_SAMPLE_SPACING);
    if (!valid) {
        return false;
    }
    return valid;
}


bool CartTrajPlanner::jspace_path_planner_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    Eigen::VectorXd qx_end(NJNTS); // need to convert to this type
    std::vector<Eigen::VectorXd> q_solns;
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;

    single_layer_nodes.clear();
    single_layer_nodes.push_back(q_start);
    

    cout << "jspace planner to Cartesian goal: " << endl;
    //    int ik_solve(Eigen::Affine3d const& desired_hand_pose,vector<Eigen::VectorXd> &q_ik_solns);
    int nsolns = ik_solver_.ik_solve(a_flange_end, q_solns);
    std::cout << "nsolns at goal pose = " << nsolns << endl;
    single_layer_nodes.clear();
    if (nsolns<1) return false; // give up
    //else power on...
    //single_layer_nodes.resize(nsolns);
    for (int isoln = 0; isoln < nsolns; isoln++) {
         single_layer_nodes.push_back(q_solns[isoln]); 
    }
    nsolns = single_layer_nodes.size();
    ROS_INFO("found %d goal IK solns",nsolns);
    if (nsolns <1) {
        return false; //no solns
    }
    
    //ok--we have at least one  soln;
    // 
    optimal_path.clear();
    optimal_path.push_back(q_start);
    //here is where we pick which of the joint-space goal solns is best:
   
    Eigen::VectorXd dq_move(NJNTS), q_modified_start(NJNTS);
    q_modified_start = q_start;
    //q_modified_start[1] = 0; // bias preference for shoulder elevation near zero,
                             // regardless of start pose
    //cout<<"q_modified_start: "<<q_modified_start.transpose()<<endl;

    //jspace_planner_weights_ are hard-coded in constructor
    //should add a "set" function to enable user to change these weights
    double penalty_best = 1000000;
    double penalty;

    cout<<"jspace_planner_weights_: "<<jspace_planner_weights_.transpose()<<endl;    
    qx_end = single_layer_nodes[0]; //default: first soln   
    cout<<"qx_end: "<<qx_end.transpose()<<endl;
    for (int i=0;i<nsolns;i++) {
        
        dq_move = q_modified_start-single_layer_nodes[i];
        cout<<"dq_move: "<<dq_move.transpose()<<endl;
        penalty=0.0;
        for (int j=0;j<NJNTS;j++) {
            penalty+= jspace_planner_weights_[j]*fabs(dq_move[j]); //should scale by speed limits
        }
        ROS_INFO("soln %d has penalty = %f",i,penalty);
        if (penalty<penalty_best) {
            penalty_best = penalty;
            qx_end = single_layer_nodes[i];
        }
    }
   
    optimal_path.push_back(qx_end);
    return true;
    //return false; // not debugged, so don't trust!
}
