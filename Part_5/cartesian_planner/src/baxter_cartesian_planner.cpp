// baxter_cartesian_planner.cpp: 
// wsn, Nov, 2016
// a library of arm-motion planning functions
// assumes use of tool-flange frame (to allow for interchangeable grippers)
// uses package joint_space_planner to find a good joint-space path among options
// from IK solutions
// key fnc:
// nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
// a_flange_des is the desired tool-flange pose of RIGHT ARM

#include <cartesian_planner/baxter_cartesian_planner.h>


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

    // define a fixed orientation corresponding to horizontal tool normal, 
    //  vector between fingers also horizontal
    // right-hand gripper approach direction along y axis
    // useful, e.g. for picking up a bottle to be poured
    tool_n_des_horiz_ << 1, 0, 0;
    tool_b_des_horiz_ << 0, 1, 0;
    tool_t_des_horiz_ = tool_b_des_horiz_.cross(tool_n_des_horiz_);
    R_gripper_horiz_.col(0) = tool_n_des_horiz_;
    R_gripper_horiz_.col(1) = tool_t_des_horiz_;
    R_gripper_horiz_.col(2) = tool_b_des_horiz_;
    
    jspace_planner_weights_.resize(7);
    jspace_planner_weights_[0] = 2;
    //penalize shoulder lift more:
    jspace_planner_weights_[1] = 10;
    //humerus:
    jspace_planner_weights_[2] = 3;
    //penalize elbow and wrist less
    jspace_planner_weights_[3] = 0.5;
    jspace_planner_weights_[4] = 0.2;
    jspace_planner_weights_[5] = 0.2;
    jspace_planner_weights_[6] = 0.2;    
}


//specify start and end poses w/rt torso.  Only orientation of end pose will be considered; orientation of start pose is ignored

bool CartTrajPlanner::cartesian_path_planner(Eigen::Affine3d a_flange_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_des.linear() = R_des;
    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples_.clear();
    //cartesian_affine_samples_.push_back(a_flange_start);
    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end - p_start;
    double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm() / dp_scalar);
    if (nsteps < 1) nsteps = 1;
    dp_vec = del_p / nsteps;
    nsteps++; //account for pose at step 0


    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep = 0; istep < nsteps; istep++) {
        a_flange_des.translation() = p_des;
        cartesian_affine_samples_.push_back(a_flange_des);

        cout << "trying: " << p_des.transpose() << endl;
        nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
        std::cout << "nsolns = " << nsolns << endl;
        single_layer_nodes.clear();
        if (nsolns > 0) {
            single_layer_nodes.resize(nsolns);
            for (int isoln = 0; isoln < nsolns; isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes);
        }
        else {
            return false;
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

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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


// alt version: specify start as a q_vec, and goal as a Cartesian pose of tool flange (w/rt torso)

bool CartTrajPlanner::cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path,
        double dp_scalar/* = CARTESIAN_PATH_SAMPLE_SPACING*/) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des, a_flange_start;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
    cout << "fwd kin from q_start: " << a_flange_start.translation().transpose() << endl;
    cout << "fwd kin from q_start R: " << endl;
    cout << a_flange_start.linear() << endl;
    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples_.clear();
    a_flange_des = a_flange_end;
    a_flange_start.linear() = R_des; // no interpolation of orientation; set goal orientation immediately   
    a_flange_des.linear() = R_des; //expected behavior: will try to achieve orientation first, before translating

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_des.translation();
    del_p = p_end - p_start;
    cout << "p_start: " << p_start.transpose() << endl;
    cout << "p_end: " << p_end.transpose() << endl;
    cout << "del_p: " << del_p.transpose() << endl;
    //double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm() / dp_scalar);
    if (nsteps < 1) nsteps = 1;
    dp_vec = del_p / nsteps;
    cout << "dp_vec for nsteps = " << nsteps << " is: " << dp_vec.transpose() << endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);

    std::vector<Vectorq7x1> q_solns;
    
    p_des = p_start;
    cartesian_affine_samples_.push_back(a_flange_start);

    for (int istep = 1; istep < nsteps; istep++) {
        p_des += dp_vec;
        a_flange_des.translation() = p_des;
        cartesian_affine_samples_.push_back(a_flange_des);
        cout << "trying: " << p_des.transpose() << endl;
        nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
        std::cout << "nsolns = " << nsolns << endl;
        single_layer_nodes.clear();
        if (nsolns > 0) {
            single_layer_nodes.resize(nsolns);
            for (int isoln = 0; isoln < nsolns; isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes);
        } else {
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

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

bool CartTrajPlanner::fine_cartesian_path_planner(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    //do trajectory planning w/ fine samples along Cartesian direction, but approximate IK solns:
    bool valid = cartesian_path_planner(q_start, a_flange_end, optimal_path, CARTESIAN_PATH_FINE_SAMPLE_SPACING);
    if (!valid) {
        return false;
    }
    //now refine this w/ IK iterations:
    refine_cartesian_path_plan(optimal_path);
    return valid;
}


// alt version: specify start as a q_vec, and desired delta-p Cartesian motion while holding R fixed

bool CartTrajPlanner::cartesian_path_planner_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    //store a vector of Cartesian affine samples for desired path:
    cartesian_affine_samples_.clear();
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des, a_flange_start, a_flange_end;
    Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;

    a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
    Eigen::Matrix3d R_des = a_flange_start.linear();
    p_des = a_flange_start.translation();
    cartesian_affine_samples_.push_back(a_flange_start);
    cout << "fwd kin from q_start p: " << p_des.transpose() << endl;
    cout << "fwd kin from q_start R: " << endl;
    cout << a_flange_start.linear() << endl;

    //a_flange_start.linear() = R_des; // override the orientation component--require point down    
    //construct a goal pose, based on start pose:
    a_flange_end.linear() = R_des;
    a_flange_des.linear() = R_des; // variable used to hold steps of a_des...always with same R
    a_flange_end.translation() = p_des + delta_p;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;

    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end - p_start; // SHOULD be same as delta_p input
    cout << "p_start: " << p_start.transpose() << endl;
    cout << "p_end: " << p_end.transpose() << endl;
    cout << "del_p: " << del_p.transpose() << endl;
    double dp_scalar = 0.05;
    nsteps = round(del_p.norm() / dp_scalar);
    if (nsteps < 1) nsteps = 1;
    dp_vec = del_p / nsteps;
    cout << "dp_vec for nsteps = " << nsteps << " is: " << dp_vec.transpose() << endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);

    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep = 1; istep < nsteps; istep++) {
        p_des += dp_vec;
        a_flange_des.translation() = p_des;
        cartesian_affine_samples_.push_back(a_flange_des);
        cout << "trying: " << p_des.transpose() << endl;
        nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
        std::cout << "nsolns = " << nsolns << endl;
        single_layer_nodes.clear();
        if (nsolns > 0) {
            single_layer_nodes.resize(nsolns);
            for (int isoln = 0; isoln < nsolns; isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes);
        } else {
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

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

//given an approximate motion plan in joint space, and given the corresponding Cartesian affine samples in cartesian_affine_samples_,
// use Jacobian iterations to improve the joint-space solution accuracy for each point;
// update optimal_path with these revised joint-space solutions

bool CartTrajPlanner::refine_cartesian_path_plan(std::vector<Eigen::VectorXd> &optimal_path) {
    int nsamps_path = optimal_path.size();

    int nsamps_cart = cartesian_affine_samples_.size();

    if (nsamps_path != nsamps_cart) {
        ROS_WARN("found %d samples in provided optimal path", nsamps_path);
        ROS_WARN("found %d samples in internal vector of Cartesian samples", nsamps_cart);
        ROS_WARN("number of jspace samples does not match number of cartesian samples; cannot refine path:");
        return false;
    }
    if (nsamps_path<2) {
        ROS_WARN("refine_cartesian path: not enough points in provided path");
        return false;
    }
    //if here, march through each solution and refine it:
    ROS_INFO("refining cartesian solutions...");
    Eigen::Affine3d des_flange_affine;
    Eigen::VectorXd approx_jspace_soln;
    Eigen::VectorXd refined_jspace_soln;
    Vectorq7x1 q_in, q_7dof_precise;
    bool valid;
    //start from i=1; keep start pose as-is
    for (int i = 1; i < nsamps_cart; i++) {
        des_flange_affine = cartesian_affine_samples_[i];
        approx_jspace_soln = optimal_path[i];
        q_in = approx_jspace_soln; //convert data type
        valid = baxter_IK_solver_.improve_7dof_soln_wrt_torso(des_flange_affine, q_in, q_7dof_precise);
        if (valid) { //note: if solution is not improved, retain approximate solution
            refined_jspace_soln = q_7dof_precise;
            optimal_path[i] = refined_jspace_soln; //install the improved soln
        }
    }
    return true;
}


// alt version: specify start as a q_vec, and goal as a Cartesian pose (w/rt torso)--but only plan a wrist path

bool CartTrajPlanner::cartesian_path_planner_wrist(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des, a_flange_start;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_start = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_start);
    cout << "fwd kin from q_start: " << a_flange_start.translation().transpose() << endl;
    cout << "fwd kin from q_start R: " << endl;
    cout << a_flange_start.linear() << endl;

    a_flange_start.linear() = R_des; // override the orientation component--require point down    
    a_flange_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des, dp_vec, del_p, p_start, p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end - p_start;
    cout << "p_start: " << p_start.transpose() << endl;
    cout << "p_end: " << p_end.transpose() << endl;
    cout << "del_p: " << del_p.transpose() << endl;
    double dp_scalar = CARTESIAN_PATH_SAMPLE_SPACING;
    nsteps = round(del_p.norm() / dp_scalar);
    if (nsteps < 1) nsteps = 1;
    dp_vec = del_p / nsteps;
    cout << "dp_vec for nsteps = " << nsteps << " is: " << dp_vec.transpose() << endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);

    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep = 1; istep < nsteps; istep++) {
        p_des += dp_vec;
        a_flange_des.translation() = p_des;
        cout << "trying: " << p_des.transpose() << endl;
        //int ik_wristpt_solve_approx_wrt_torso(Eigen::Affine3d const& desired_hand_pose_wrt_torso,std::vector<Vectorq7x1> &q_solns); 
        nsolns = baxter_IK_solver_.ik_wristpt_solve_approx_wrt_torso(a_flange_des, q_solns);
        std::cout << "nsolns = " << nsolns << endl;
        single_layer_nodes.clear();
        if (nsolns > 0) {
            single_layer_nodes.resize(nsolns);
            for (int isoln = 0; isoln < nsolns; isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes);
        } else {
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
 
    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, jspace_planner_weights_);
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

//bool CartTrajPlanner::cartesian_path_planner(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {

bool CartTrajPlanner::jspace_trivial_path_planner(Vectorq7x1 q_start, Vectorq7x1 q_end, std::vector<Eigen::VectorXd> &optimal_path) {
    Eigen::VectorXd qx_start(7), qx_end(7); // need to convert to this type
    //qx_start<<0,0,0,0,0,0,0; // resize
    //qx_end<<0,0,0,0,0,0,0;       
    for (int i = 0; i < 7; i++) {
        qx_start[i] = q_start[i];
        qx_end[i] = q_end[i];
    }
    cout << "jspace_trivial_path_planner: " << endl;
    cout << "q_start: " << qx_start.transpose() << endl;
    cout << "q_end: " << qx_end.transpose() << endl;
    optimal_path.clear();
    optimal_path.push_back(qx_start);
    optimal_path.push_back(qx_end);
    return true;
}

bool CartTrajPlanner::jspace_path_planner_to_affine_goal(Vectorq7x1 q_start, Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    Eigen::VectorXd qx_start(7), qx_end(7); // need to convert to this type
    std::vector<Vectorq7x1> q_solns;
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node, precise_node; 
    Vectorq7x1 q_approx,q_refined;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    
    for (int i = 0; i < 7; i++) {
        qx_start[i] = q_start[i];
    }
    cout << "jspace planner to Cartesian goal: " << endl;
    
    int nsolns = baxter_IK_solver_.ik_solve_approx_wrt_torso(a_flange_end, q_solns);
    std::cout << "nsolns at goal pose = " << nsolns << endl;
    single_layer_nodes.clear();
    if (nsolns<1) return false; // give up
    //else power on...
    //single_layer_nodes.resize(nsolns);
    for (int isoln = 0; isoln < nsolns; isoln++) {
         q_approx = q_solns[isoln];
        if (baxter_IK_solver_.improve_7dof_soln_wrt_torso(a_flange_end, q_approx, q_refined)) {
            precise_node = q_refined; //convert to Eigen::VectorXd
            cout<<"precise_node: "<<precise_node.transpose()<<endl;
            single_layer_nodes.push_back(precise_node);    
        }
    }
    nsolns = single_layer_nodes.size();
    ROS_INFO("found %d refined goal IK solns",nsolns);
    if (nsolns <1) {
        return false; //no solns
    }
    

    //ok--we have at least one precise soln;
    // 
    optimal_path.clear();
    optimal_path.push_back(qx_start);
    //here is where we pick which of the joint-space goal solns is best:
    // note: baxter's joints all include "0" within the reachable range
    // and all solns must pass through 0.  Therefore, qx_end-qx_start is an appropriate metric
    // and motion THRU zero is appropriate (not shortest periodic distance)
    
    //note: prepose hard-code defined in baxter_cart_move_as;
    //q_pre_pose_ << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;
    // should move this to a header;  want shoulder elevation near zero to elevate elbow

    Eigen::VectorXd dq_move(7), q_modified_start(7);
    q_modified_start = qx_start;
    q_modified_start[1] = 0; // bias preference for shoulder elevation near zero,
                             // regardless of start pose
    cout<<"q_modified_start: "<<q_modified_start.transpose()<<endl;
    //from baxter_traj_streamer.h:
    //const double q0dotmax = 0.5;
    //const double q1dotmax = 0.5;
    //const double q2dotmax = 0.5;
    //const double q3dotmax = 0.5;
    //const double q4dotmax = 1;
    //const double q5dotmax = 1;
    //const double q6dotmax = 1;
    // should make these speed limits more accessible
    //jspace_planner_weights_ are defined here, above
    double penalty_best = 1000000;
    double penalty;

    cout<<"jspace_planner_weights_: "<<jspace_planner_weights_.transpose()<<endl;    
    qx_end = single_layer_nodes[0]; //default: first soln   
    cout<<"qx_end: "<<qx_end.transpose()<<endl;
    for (int i=0;i<nsolns;i++) {
        
        dq_move = q_modified_start-single_layer_nodes[i];
        cout<<"dq_move: "<<dq_move.transpose()<<endl;
        penalty=0.0;
        for (int j=0;j<7;j++) {
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



// use this classes baxter fk solver to compute and return tool-flange pose w/rt torso, given right-arm joint angles

Eigen::Affine3d CartTrajPlanner::get_fk_Affine_from_qvec(Vectorq7x1 q_vec) {
    Eigen::Affine3d Affine_pose;
    Affine_pose = baxter_fwd_solver_.fwd_kin_flange_wrt_torso_solve(q_vec);

}

