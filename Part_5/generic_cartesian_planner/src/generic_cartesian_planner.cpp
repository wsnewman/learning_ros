// generic cartesian planner (not including collision considerations)
// wsn, Dec, 2017
// a library of arm-motion planning functions
// uses package joint_space_planner to find a good joint-space path among options
// from IK solutions

// for this planner, the robot to which it applies depends on the fk_ik library
//  with which it is compiled; see example in package irb120_planner

#include <generic_cartesian_planner/generic_cartesian_planner.h>


//constructor: CartTrajPlanner(IKSolver * pIKSolver, FwdSolver * pFwdSolver);
CartTrajPlanner::CartTrajPlanner(IKSolver * pIKSolver_arg, FwdSolver * pFwdSolver_arg, int njnts) // optionally w/ args, e.g. 
//: as_(nh_, "CartTrajPlanner", boost::bind(&cartTrajActionServer::executeCB, this, _1), false)
{
    ROS_INFO("in constructor of CartTrajPlanner...");
    NJNTS_=njnts; // default; parent code should reset this w/ "set" member fnc
    pIKSolver_ = pIKSolver_arg;
    pFwdSolver_ = pFwdSolver_arg; //instantiate a forward-kinematics solver   
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
    
    jspace_planner_weights_.resize(NJNTS_); //planner weights; get defaults from robot header
    //default planner weights; reset these w/ set_jspace_planner_weights()
    for (int i=0;i<NJNTS_;i++)  jspace_planner_weights_[i] = 1.0;
    
    //default path sampling values:
    cartesian_path_sample_spacing_ = CARTESIAN_PATH_SAMPLE_SPACING;
    cartesian_path_fine_sample_spacing_ = CARTESIAN_PATH_FINE_SAMPLE_SPACING;
}

//    void set_jspace_planner_weights(vector<double> planner_joint_weights); //Eigen::VectorXd jspace_planner_weights);
void CartTrajPlanner::set_jspace_planner_weights(vector<double> planner_joint_weights) {
    jspace_planner_weights_.resize(NJNTS_);
    for (int i=0;i<NJNTS_;i++) {
       jspace_planner_weights_[i] = planner_joint_weights[i]; 
    }
};

void CartTrajPlanner::set_joint_names(vector<string> jnt_names) {
    jnt_names_.resize(NJNTS_);
    for (int i=0;i<NJNTS_;i++) {
        jnt_names_[i] = jnt_names[i];
    }
}


//given a path in joint space, assign arrival times and fill out a trajectory message
//take argument "arrival_time" and assign intermediate arrival times in equal intervals to path points
// does not check if resulting joint velocities are feasible
void CartTrajPlanner::path_to_traj(std::vector<Eigen::VectorXd> qvecs, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    trajectory_point.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < NJNTS_; i++) {
        new_trajectory.joint_names.push_back(jnt_names_[i].c_str());
    }

    //try imposing a time delay on first point to work around ros_controller complaints
    double t_start=0.001; //0.05;

    new_trajectory.header.stamp = ros::Time::now(); //+ros::Duration(t_start);  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = t_start;
    q_start = qvecs[0];
    q_end = qvecs[0];
    int npts = qvecs.size();
    del_time = arrival_time/(npts-1);
    //cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");

    trajectory_point.positions.clear();
    trajectory_point.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < NJNTS_; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        //q_start = q_end;
        q_end = qvecs[iq];
        //dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        //del_time = min_transition_time(dqvec);
        //if (del_time < min_dt_traj_)
        //    del_time = min_dt_traj_;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        
        
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < NJNTS_; i++) { //copy over the joint-command values
            trajectory_point.positions[i] = q_end[i];
        }
        trajectory_point.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point);
    }
    /*
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<NJNTS_;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }*/
}

//this version starts from affine and goes to affine, i.e. start pose is not specified
bool CartTrajPlanner::cartesian_path_planner_w_rot_interp(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::VectorXd> &optimal_path) {
    
    //sample nsteps along a Cartesian path from specified start to end Affine poses; interpolate orientation with angle/axis
    // use the CartesianInterpolator class to help; populates the std::vector of cartesian_affine_samples_
    ROS_INFO("interpolating Cartesian path: ");
    cartesianInterpolator_.cartesian_path_planner_w_rot_interp(a_flange_start,a_flange_end, nsteps,cartesian_affine_samples_);
    
     //now have a vector of Cartesian samples in cartesian_affine_samples_
     //for each such sample, compute IK options to set up the network to be solved
    int nsamps = cartesian_affine_samples_.size();
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des;
    int nsolns;
    std::vector<Eigen::VectorXd> q_solns;
    ROS_INFO("computing IK options");
    for (int istep=0;istep<nsamps;istep++) 
    {
            a_flange_des = cartesian_affine_samples_[istep];
            cout<<"a_flange Origin: "<<a_flange_des.translation().transpose()<<endl;
            cout<<"a_flange orientation: "<<endl;
            cout<<a_flange_des.linear()<<endl;
            //pFwdSolver_, pIKSolver_
            //nsolns = pIKSolver_->ik_solve(a_flange_des, q_solns); //uses whatever ik_solver_ has been provided
            nsolns = pIKSolver_->ik_solve(a_flange_des, q_solns); //uses whatever ik_solver_ has been provided
            ROS_INFO("istep = %d; nsolns = %d",istep,nsolns);

            //std::cout<<"cartesian step "<<istep<<" has = "<<nsolns<<" IK solns"<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    single_layer_nodes[isoln] = q_solns[isoln]; //node;
                }
                path_options.push_back(single_layer_nodes);
            }
         else {
            ROS_WARN("no valid IK soln...");
            return false;
          }

    }
    //path_options is now a feedforward network; use dynamic programming to find min cost through network:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }

    optimal_path.resize(nlayers);
    double trip_cost;

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, jspace_planner_weights_);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }
    //given jspace path, refine it, in case IK requires addl numerical correction:
    pIKSolver_->ik_refine(cartesian_affine_samples_, optimal_path);

    //now, jsp is deleted, but optimal_path lives on:
    /*
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
     * */
    return true;
}

   bool CartTrajPlanner::jspace_trivial_path_planner(Eigen::VectorXd q_start,Eigen::VectorXd q_end,std::vector<Eigen::VectorXd> &optimal_path) {
       cout<<"jspace_trivial_path_planner: "<<endl;
       cout<<"q_start: "<<q_start.transpose()<<endl;
       cout<<"q_end: "<<q_end.transpose()<<endl;
       optimal_path.clear();
       optimal_path.push_back(q_start);
       optimal_path.push_back(q_end);
       return true;
   }
   
  // trajectory_msgs::JointTrajectory des_trajectory_;
bool CartTrajPlanner::plan_jspace_traj_qstart_to_qend(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &trajectory) {
           Eigen::VectorXd dq_move(NJNTS_), q_step(NJNTS_);
      dq_move = (q_goal-q_start)/nsteps;
      //fill a path:
      std::vector<Eigen::VectorXd> path;
      path.clear();
      path.push_back(q_start);
      q_step  = q_start;
      for (int i=0;i<nsteps;i++) {
          q_step = q_step+dq_move;
          path.push_back(q_step);
      }
      
     path_to_traj(path, arrival_time, trajectory);
 
    return true;
    }
   

   //    bool plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);

bool CartTrajPlanner::plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory) {
  std::vector<Eigen::VectorXd> path;
  bool path_ok;
  path_ok= plan_jspace_path_qstart_to_des_flange_affine(q_start, nsteps, a_flange_end, path);
  if (!path_ok) return false;
  path_to_traj(path, arrival_time, new_trajectory);
    return true;
}

   //    bool jspace_path_planner_to_affine_goal(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, int nsamps, double arrival_time, std::vector<Eigen::VectorXd> &optimal_path);
//    bool plan_jspace_traj_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory);
//    bool plan_jspace_path_qstart_to_affine_goal(Eigen::VectorXd  q_start, Eigen::Affine3d a_flange_end, int nsteps, std::vector<Eigen::VectorXd> &optimal_path);

bool CartTrajPlanner::plan_jspace_path_qstart_to_des_flange_affine(Eigen::VectorXd  q_start, int nsteps, Eigen::Affine3d goal_flange_affine,std::vector<Eigen::VectorXd> &optimal_path){
    Eigen::VectorXd qx_end(NJNTS_); // need to convert to this type
    std::vector<Eigen::VectorXd> q_solns;
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;

    single_layer_nodes.clear();
    single_layer_nodes.push_back(q_start);
    

    cout << "jspace planner to Cartesian goal: " << endl;
    //    int ik_solve(Eigen::Affine3d const& desired_hand_pose,vector<Eigen::VectorXd> &q_ik_solns);
    int nsolns = pIKSolver_->ik_solve(goal_flange_affine, q_solns);
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
   
    Eigen::VectorXd dq_move(NJNTS_), q_step(NJNTS_), q_modified_start(NJNTS_);
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
        for (int j=0;j<NJNTS_;j++) {
            penalty+= jspace_planner_weights_[j]*fabs(dq_move[j]); //should scale by speed limits
        }
        ROS_INFO("soln %d has penalty = %f",i,penalty);
        if (penalty<penalty_best) {
            penalty_best = penalty;
            qx_end = single_layer_nodes[i];
        }
    }
    //have q_start and qx_end; chop this up into npts steps
    q_step = q_start;
    dq_move = (qx_end - q_start)/nsteps;
    for (int i=0;i<nsteps;i++) {
        q_step = q_step + dq_move;
        optimal_path.push_back(q_step);
    }
   
    
    return true;
    //return false; // not debugged, so don't trust!
}

//Cartesian trajectory planner:
bool CartTrajPlanner::plan_cartesian_traj_qstart_to_des_flange_affine(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_goal,
        int nsteps,  double arrival_time, trajectory_msgs::JointTrajectory &new_trajectory) {
     std::vector<Eigen::VectorXd> path;
     bool valid_path = plan_cartesian_path_w_rot_interp(q_start,a_flange_goal, nsteps,path);
     if (!valid_path) return false;

     path_to_traj(path, arrival_time, new_trajectory); 
     return true;
        
  }

    
    
//Cartesian planner that interpolates both translation and rotation--using angle/axis interpolation--
// starting from a specified joint-space pose and ending with a specified Cartesian-space pose
// specify goal pose w/rt toolflange
bool CartTrajPlanner::plan_cartesian_path_w_rot_interp(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_end, 
        int nsteps,  std::vector<Eigen::VectorXd> &optimal_path) {

    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des,a_flange_start;

    cout<<"plan_cartesian_path_w_rot_interp"<<endl;
    cout<<"q_start = "<<q_start.transpose()<<endl;
     a_flange_start = pFwdSolver_->fwd_kin_solve(q_start);
     cout<<"a_flange_start origin: "<<a_flange_start.translation().transpose()<<endl;
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


    std::vector<Eigen::VectorXd> q_solns;
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
    single_layer_nodes.clear();
    single_layer_nodes.resize(1); //coerce starting point to be q_start;
    single_layer_nodes[0] = q_start; //starting layer has a single node;
    path_options.push_back(single_layer_nodes);
                
    for (int istep=1;istep<nsamps;istep++) 
    {
            a_flange_des = cartesian_affine_samples_[istep];
            nsolns = pIKSolver_->ik_solve(a_flange_des, q_solns);
            std::cout<<"cartesian step "<<istep<<" has = "<<nsolns<<" IK solns"<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    single_layer_nodes[isoln] = q_solns[isoln]; //node;
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
    weights.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
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
    Eigen::Affine3d affine_fk;
    Eigen::Vector3d origin_fk;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
        affine_fk = pFwdSolver_->fwd_kin_solve(optimal_path[ilayer]);
        origin_fk=affine_fk.translation();
        cout<<"fk origin: "<<origin_fk.transpose()<<endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    //refine this path, if necessary:
    //given jspace path, refine it, in case IK requires addl numerical correction:
    pIKSolver_->ik_refine(cartesian_affine_samples_, optimal_path);
    return true;
}

/*

bool CartTrajPlanner::cartesian_path_planner(Eigen::VectorXd q_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    return cartesian_path_planner(q_start, a_flange_end, optimal_path, cartesian_path_sample_spacing_);
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
    
    int nsteps
 
    
   cartesianInterpolator_.cartesian_path_planner_w_rot_interp(a_flange_start,a_flange_end, 
        int nsteps,  std::vector<Eigen::Affine3d> &cartesian_affine_samples)

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
    double dp_scalar = cartesian_path_sample_spacing_;
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
            nsolns = pIKSolver_->ik_solve(a_flange_des, q_solns);
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
    weights.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
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
        affine_fk = pFwdSolver_->fwd_kin_solve(q_solns[isoln]);
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





bool CartTrajPlanner::multipoint_cartesian_path_planner(std::vector<Eigen::Affine3d> a_flange_poses,std::vector<int> nsteps_vec, 
std::vector<Eigen::VectorXd> &optimal_path, std::vector<int> &nsteps_to_via_pt) {
  Eigen::Affine3d a_start,a_end;
  Eigen::vectorXd q_start_of_next_segment;
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
     int npts = optimal_path.size();
     q_start_of_next_segment = optimal_path[npts-1]; // prevent jspace jumps at via pts
     
     a_start = a_end;
     
     a_end = a_flange_poses[ivia];
     nsamp_pts = nsteps_vec[ivia-1];
     //plan from specified start jspace pose to subgoal flange pose using nsamp_pts interpolation values
     //return this segment's solution in partial_path
     good_path = cartesian_path_planner_w_rot_interp(q_start_of_next_segment,a_end, nsamp_pts,  partial_path);
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


//as above, but with requirement on first jspace pose:
//for a_flange_poses, DO start w/ fk(q_start)
//nsteps_vec[i] specifies number of desired sample points from pose i to pose i+1
bool CartTrajPlanner::multipoint_cartesian_path_planner(Eigen::VectorXd q_start,
        std::vector<Eigen::Affine3d> a_flange_poses,std::vector<int> nsteps_vec, 
std::vector<Eigen::VectorXd> &optimal_path, std::vector<int> &nsteps_to_via_pt) {
  Eigen::Affine3d a_start,a_end;
  Eigen::VectorXd q_start_of_next_segment;
  q_start_of_next_segment = q_start;
  int nsamp_pts;
  std::vector<Eigen::VectorXd> partial_path;
  int n_via_pts;
  n_via_pts = a_flange_poses.size();
  if (n_via_pts<1) {
    ROS_WARN("too few poses for path planning");
    return false;
  }
  a_start = a_flange_poses[0]; //user must supply an initial pose, or at least a filler
  a_end = a_flange_poses[1];
  nsamp_pts = nsteps_vec[0];
  bool good_path;
  //good_path = cartesian_path_planner_w_rot_interp(a_start, a_end, nsamp_pts,  partial_path);
  good_path = cartesian_path_planner_w_rot_interp(q_start_of_next_segment,a_end, nsamp_pts,  partial_path);
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
     int npts = optimal_path.size();
     q_start_of_next_segment = optimal_path[npts-1]; // prevent jspace jumps at via pts     
     //a_start = a_end;    
     a_end = a_flange_poses[ivia];
     nsamp_pts = nsteps_vec[ivia-1];
     //plan from specified start jspace pose to subgoal flange pose using nsamp_pts interpolation values
     //return this segment's solution in partial_path
     good_path = cartesian_path_planner_w_rot_interp(q_start_of_next_segment,a_end, nsamp_pts,  partial_path);
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
    a_tool_start = pFwdSolver_->fwd_kin_solve(q_start);
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
            nsolns = pIKSolver_->ik_solve(a_tool_des, q_solns);
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
    weights.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
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

    a_tool_start = pFwdSolver_->fwd_kin_solve(q_start);
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
            nsolns = pIKSolver_->ik_solve(a_tool_des, q_solns);
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
    weights.resize(NJNTS_);
    for (int i = 0; i < NJNTS_; i++) {
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



   
   //this version uses a finer Cartesian sampling dp than the default

bool CartTrajPlanner::fine_cartesian_path_planner(Eigen::VectorXd q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    //do trajectory planning w/ fine samples along Cartesian direction, but approximate IK solns:
    bool valid = cartesian_path_planner(q_start, a_flange_end, optimal_path, cartesian_path_fine_sample_spacing_);
    if (!valid) {
        return false;
    }
    return valid;
}



*/