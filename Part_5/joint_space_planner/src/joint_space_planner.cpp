// wsn, October, 2014
// joint-space planner, organized as a class

#include <joint_space_planner/joint_space_planner.h>
using namespace std;

JointSpacePlanner::JointSpacePlanner(int i, int j) {
    //cout<<"dummy constructor, i,j = "<<i<<","<<j<<endl; 
}

// can initialize member vars during their construction if list these as initializers, as below
//JointSpacePlanner::JointSpacePlanner(vector<vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights): path_options_(path_options),penalty_weights_(weights) {
JointSpacePlanner::JointSpacePlanner(vector<vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights): penalty_weights_(weights) {
 
    // make a pointer member variable
    path_options_ptr_ = &path_options; 
    
    
    // tell me how many weights there are, and I'll tell you the dimension of the pose states being evaluated
    //weights_= weights;
    problem_dimension_ = weights.size(); // this will be the dimension of the Eigen::VectorXd poses
    //penalty_weights_= weights; // copy to private var; already done via initialization list
  
    cout<<"vector size: "<<problem_dimension_<<endl;
    nlayers_ = path_options.size();
    cout<<"num layers = "<<nlayers_<<endl;
            //walk through these and resize their guts:
    all_costs_.resize(nlayers_);
    next_indices_.resize(nlayers_);
    for (int i=0;i<nlayers_;i++) {
        int size_ilayer = path_options[i].size();
        cout<<"ilayer: "<<i<<"; # options = "<<size_ilayer<<endl;
        all_costs_[i].resize(size_ilayer);
        next_indices_[i].resize(size_ilayer);
    }
    cout<<"calling constructor helper: "<<endl;
    constructor_helper_();  
    // make similarly-sized holders for costs and for indices;
    
     //compute all costs by passing reference arg path_options into function
     // somewhat odd that this major computation would be done in a constructor, though.
     // further, would need to follow this up with call to compute_optimal_path() also getting the path_options arg;
     // so...ALL the work would be done in the constructor!
     
     cout<<"computing min costs from constructor"<<endl;
     compute_all_min_costs(path_options); // do this in constructor, so can use input reference arg without recopying it
     
     cout<<"computing optimal path in constructor"<<endl;
     compute_optimal_path(path_options); // answer will be in optimal_path_
     cout<<"done with constructor"<<endl;
} 

//alternative constructor: pass in additional reference object augment the trip costs:
// use this object to evaluate attractive cutting poses with respect to compliance effectiveness of humerous
// can initialize member vars during their construction if list these as initializers, as below
//JointSpacePlanner::JointSpacePlanner(vector<vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights): path_options_(path_options),penalty_weights_(weights) {
JointSpacePlanner::JointSpacePlanner(vector<vector<Eigen::VectorXd> > &path_options,Eigen::VectorXd weights, vector<vector<double> > &humerus_sensitivities): penalty_weights_(weights) {
 
    // next line already done w/ initialization list;
     //path_options_ = path_options; // copy?  this could be expensive; maybe just work w/ path_options?   
    
    // however, probably want a call by reference, as this copy can be slow
    // make a pointer member variable?
    path_options_ptr_ = &path_options; // maybe something like this would avoid the copy?
    
    
    // tell me how many weights there are, and I'll tell you the dimension of the pose states being evaluated
    //weights_= weights;
    problem_dimension_ = weights.size(); // this will be the dimension of the Eigen::VectorXd poses
    //penalty_weights_= weights; // copy to private var; already done via initialization list
  
    cout<<"vector size: "<<problem_dimension_<<endl;
    nlayers_ = path_options.size();
    cout<<"num layers = "<<nlayers_<<endl;
    
    
        //walk through these and resize their guts:
    for (int i=0;i<nlayers_;i++) {
        int size_ilayer = path_options[i].size();
        all_costs_[i].resize(size_ilayer);
        next_indices_[i].resize(size_ilayer);
    }
    
    constructor_helper_();  
    // make similarly-sized holders for costs and for indices;
    
     //compute all costs by passing reference arg path_options into function
     // somewhat odd that this major computation would be done in a constructor, though.
     // further, would need to follow this up with call to compute_optimal_path() also getting the path_options arg;
     // so...ALL the work would be done in the constructor!
     
     cout<<"computing min costs from constructor"<<endl;
     compute_all_min_costs(path_options); // do this in constructor, so can use input reference arg without recopying it
     
     //add in influence of compliance sensitivities of humerus:
     //augment_all_min_costs_(humerus_sensitivities);
     
     cout<<"computing optimal path in constructor"<<endl;
     compute_optimal_path(path_options); // answer will be in optimal_path_
     cout<<"done with constructor"<<endl;
}

void JointSpacePlanner::augment_all_min_costs_(vector<vector<double> > &humerus_sensitivities) {
    //humerus_sensitivities should have same dimensions as path_options
    // UNFINISHED
}

void JointSpacePlanner::constructor_helper_() {
    
    // make similarly-sized holders for costs and for indices;
    all_costs_.resize(nlayers_);
    next_indices_.resize(nlayers_);
    

    
    //fill the last (destination) cost layer with 0's
    int size_last_layer = all_costs_[nlayers_-1].size();
    cout<<"last layer has num options = "<<size_last_layer<<endl;
    for (int i=0;i<size_last_layer;i++) {
        all_costs_[nlayers_-1][i]=0.0; //cost-to-go is zero at all end nodes
        next_indices_[nlayers_-1][i]= -1; //next index is invalid at end nodes
    }
    
    // resize the solution vector to hold resulting path--std::vector of Eigen::VectorXd vecs
     optimal_path_.resize(nlayers_);   
     vec_of_zeros_.resize(problem_dimension_);
     diff_.resize(problem_dimension_);
     diff_sqd_.resize(problem_dimension_);

     //let's pre-fill the optimal path with dummy vectors of appropriate size:
     for (int i=0;i<nlayers_;i++) {
         optimal_path_[i] = vec_of_zeros_;
     }
        
}
//// copy solution into provided container, "optimal_path"
    void JointSpacePlanner::get_soln(std::vector<Eigen::VectorXd> &optimal_path) { 
        for (int ilayer=0;ilayer<nlayers_;ilayer++) {
             optimal_path[ilayer] = optimal_path_[ilayer];
        }
    }
    
    
// compute incremental cost to go from pose1 to pose2, weighted, possibly squared
double JointSpacePlanner::score_move(Eigen::VectorXd pose1, Eigen::VectorXd pose2)  {
    diff_ = pose1-pose2;
    for (int i=0; i<problem_dimension_;i++) {
       diff_sqd_(i) = diff_(i)*diff_(i);
    }
    double penalty = penalty_weights_.dot(diff_sqd_);
    
    return penalty;
}

//working backwards from final-state options, compute min costs and associated optimal options for all layers
bool JointSpacePlanner::compute_all_min_costs() {
    for (int i_layer = nlayers_-1;i_layer>0; i_layer--) {
        find_best_moves_single_layer(i_layer);
    }
    cout<<"done computing all costs"<<endl;
    
    return true;
}

//alternative version: pass in reference to all_costs
bool JointSpacePlanner::compute_all_min_costs(vector<vector<Eigen::VectorXd> > &path_options) {
    for (int i_layer = nlayers_-1;i_layer>0; i_layer--) {
        int target_options = path_options[i_layer].size();
        int prior_options = path_options[i_layer-1].size();
        cout<<"compute_all_min_costs, layer: "<<i_layer<<" has "<<target_options<<" target options and "<<prior_options<<" prior options"<<endl;
        find_best_moves_single_layer(path_options,i_layer);
    }
    // cout<<"done computing all costs"<<endl;
    
    return true;
}


//incremental computation: for a given target layer, assuming this target layer has all of its downstream cost-to-go values filled in,
// compute the min cost-to-go values (and corresponding optimal move indices) for the prior layer
bool JointSpacePlanner::find_best_moves_single_layer(int target_layer_index) {
    double cost_to_go;
    double inc_cost_to_go;
    //min_cost_to_go_ = 0.0; // put in best value here

    //step through all poses in layer k-1;
    cout<<"find_best_moves_single_layer: target layer: "<<target_layer_index<<endl;
    prior_pose_options_ = path_options_[target_layer_index-1]; //try pointer access..
    int n_prior_poses = prior_pose_options_.size();   
    cout<<"n_prior_poses= "<<n_prior_poses<<endl;

    
    next_pose_options_ = path_options_[target_layer_index];
    int n_target_poses = next_pose_options_.size();
    cout<<"n_target_poses= "<<n_target_poses<<endl;
    target_pose_costs_to_go_ = all_costs_[target_layer_index];
    
    int j_target_0;
    for (int i_prior_pose=0;i_prior_pose<n_prior_poses;i_prior_pose++) {
       //cout<<"i_prior_pose = "<<i_prior_pose<<endl;
        prior_pose_ = prior_pose_options_[i_prior_pose];
        //initialize the search for lowest cost-to-go, and corresponding move index:
        move_index_min_cost_to_go_ = 0; // put in best index to next move here; assume index 0 to start; 
        j_target_0 = move_index_min_cost_to_go_;
        target_pose_ = next_pose_options_[j_target_0]; // initialize search with first target option;
        inc_cost_to_go = score_move(prior_pose_,target_pose_);
        cost_to_go =  target_pose_costs_to_go_[j_target_0] + inc_cost_to_go;
        min_cost_to_go_ = cost_to_go; // initialize minimization search


        cout<<"target layer "<<target_layer_index<<"; i_prior_pose: "<<i_prior_pose<<"; j_target: "<<j_target_0 <<endl;
        cout<<"prior pose = "<<prior_pose_.transpose()<<endl; // DEBUG        
        cout<<"target pose: "<<target_pose_.transpose()<<endl;
        cout<<"target cost-to-go = "<<target_pose_costs_to_go_[j_target_0]<<"; inc_cost_to_go = "<<inc_cost_to_go<<endl;       
        // now explore all other target options, for this prior pose, and find best choice:
        for (int j_target=1;j_target<n_target_poses;j_target++) {
            target_pose_ = next_pose_options_[j_target];
            inc_cost_to_go = score_move(prior_pose_,target_pose_);
            cost_to_go = target_pose_costs_to_go_[j_target] + inc_cost_to_go;
            cout<<"target layer "<<target_layer_index<<"; i_prior_pose: "<<i_prior_pose<<"; j_target: "<<j_target<<endl;           
            cout<<"target pose: "<<target_pose_.transpose()<<endl;
            cout<<"target cost-to-go = "<<target_pose_costs_to_go_[j_target]<<"; inc_cost_to_go = "<<inc_cost_to_go<<endl;
            if (cost_to_go< min_cost_to_go_) { // found a better option...
                min_cost_to_go_ = cost_to_go;
                move_index_min_cost_to_go_ = j_target;
            }
        }
        //when here, have found the best choice starting from prior_pose_ = prior_pose_options_[i_prior_pose];
        // install the result:
        all_costs_[target_layer_index-1][i_prior_pose]=min_cost_to_go_;
        next_indices_[target_layer_index-1][i_prior_pose] = move_index_min_cost_to_go_;
    }
    //get the target pose options and costs:    
    cout<<"done computing costs for target layer "<<target_layer_index<<endl;
    return true; // return true, unless there is a problem
}  

//alt version: pass in reference to all_costs
//incremental computation: for a given target layer, assuming this target layer has all of its downstream cost-to-go values filled in,
// compute the min cost-to-go values (and corresponding optimal move indices) for the prior layer
bool JointSpacePlanner::find_best_moves_single_layer(vector<vector<Eigen::VectorXd> > &path_options,int target_layer_index) {
    double cost_to_go;
    double inc_cost_to_go;
    //min_cost_to_go_ = 0.0; // put in best value here

    //step through all poses in layer k-1;
    //cout<<"find_best_moves_single_layer: target layer: "<<target_layer_index<<endl;
    prior_pose_options_ = path_options[target_layer_index-1]; //try pointer access..
    int n_prior_poses = prior_pose_options_.size();   
    //cout<<"n_prior_poses= "<<n_prior_poses<<endl;

    
    next_pose_options_ = path_options[target_layer_index];
    int n_target_poses = next_pose_options_.size();
    //cout<<"n_target_poses= "<<n_target_poses<<endl;
    target_pose_costs_to_go_ = all_costs_[target_layer_index];
    
    int j_target_0;
    for (int i_prior_pose=0;i_prior_pose<n_prior_poses;i_prior_pose++) {
       //cout<<"i_prior_pose = "<<i_prior_pose<<endl;
        prior_pose_ = prior_pose_options_[i_prior_pose];
        //initialize the search for lowest cost-to-go, and corresponding move index:
        move_index_min_cost_to_go_ = 0; // put in best index to next move here; assume index 0 to start; 
        j_target_0 = move_index_min_cost_to_go_;
        target_pose_ = next_pose_options_[j_target_0]; // initialize search with first target option;
        inc_cost_to_go = score_move(prior_pose_,target_pose_);
        cost_to_go =  target_pose_costs_to_go_[j_target_0] + inc_cost_to_go;
        min_cost_to_go_ = cost_to_go; // initialize minimization search


        //cout<<"target layer "<<target_layer_index<<"; i_prior_pose: "<<i_prior_pose<<"; j_target: "<<j_target_0 <<endl;
       //cout<<"prior pose = "<<prior_pose_.transpose()<<endl; // DEBUG        
       //cout<<"target pose: "<<target_pose_.transpose()<<endl;
        //cout<<"target cost-to-go = "<<target_pose_costs_to_go_[j_target_0]<<"; inc_cost_to_go = "<<inc_cost_to_go<<endl;       
        // now explore all other target options, for this prior pose, and find best choice:
        for (int j_target=1;j_target<n_target_poses;j_target++) {
            target_pose_ = next_pose_options_[j_target];
            inc_cost_to_go = score_move(prior_pose_,target_pose_);
            cost_to_go = target_pose_costs_to_go_[j_target] + inc_cost_to_go;
            //cout<<"target layer "<<target_layer_index<<"; i_prior_pose: "<<i_prior_pose<<"; j_target: "<<j_target<<endl;           
            //cout<<"target pose: "<<target_pose_.transpose()<<endl;
            //cout<<"target cost-to-go = "<<target_pose_costs_to_go_[j_target]<<"; inc_cost_to_go = "<<inc_cost_to_go<<endl;
            if (cost_to_go< min_cost_to_go_) { // found a better option...
                min_cost_to_go_ = cost_to_go;
                move_index_min_cost_to_go_ = j_target;
            }
        }
        //when here, have found the best choice starting from prior_pose_ = prior_pose_options_[i_prior_pose];
        // install the result:
        all_costs_[target_layer_index-1][i_prior_pose]=min_cost_to_go_;
        next_indices_[target_layer_index-1][i_prior_pose] = move_index_min_cost_to_go_;
    }
    //get the target pose options and costs:    
    //cout<<"done computing costs for target layer "<<target_layer_index<<endl;
    return true; // return true, unless there is a problem
}  

//main routine: accept a large array of options, and return the optimal path
bool JointSpacePlanner::compute_optimal_path(std::vector<Eigen::VectorXd> optimal_path ) {
    cout<<"computing all min costs"<<endl;
    //compute_all_min_costs();
    cout<<"finding optimal path"<<endl;
    double cost_to_go,min_total_trip_cost;
    double trip_cost; // debug

    // given the cost array, find the best path;
    // start by finding the optional starting node, layer 0
    int nstart_solns = all_costs_[0].size();
    int start_index=0;
    min_cost_to_go_=all_costs_[0][0];
    move_index_min_cost_to_go_=0;
    for (int istart=1;istart<nstart_solns;istart++) {
        cost_to_go = all_costs_[0][istart];
           if (cost_to_go< min_cost_to_go_) { // found a better option...
                min_cost_to_go_ = cost_to_go;
                move_index_min_cost_to_go_ = istart;
            }
    }
    trip_cost = 0;
    cout<<"entire trip min cost is "<<min_cost_to_go_<<" starting from initial node "<<move_index_min_cost_to_go_<<endl;
    min_total_trip_cost_= min_cost_to_go_; // save this as the overall result
    min_total_trip_cost =min_cost_to_go_;
    // fill up the path using optimal starting node and the identified best choices:
    optimal_path_[0] = path_options_[0][move_index_min_cost_to_go_];
    // optimal_path_[0] = path_options[0][move_index_min_cost_to_go_];   // this does not work; must use member var
    cout<<"layer 0 pose = "<<optimal_path_[0].transpose()<<endl;
    //cout<<"from layer 0, opt next node= "<<move_index_min_cost_to_go_<<endl;
    //klayer is next target layer:
    for (int klayer =1;klayer<nlayers_;klayer++) {
        //from node iopt in layer klayer-1, find the best move to layer k, per next_indices
        //from this index in klayer-1, find the corresponding pose and add it to optimal_path
        //step through the layers this way, from start to goal
        move_index_min_cost_to_go_= next_indices_[klayer-1][move_index_min_cost_to_go_];
        //cout<<"from layer "<<klayer-1<<", opt next node= "<<move_index_min_cost_to_go_<<endl;
        optimal_path_[klayer]= path_options_[klayer][move_index_min_cost_to_go_];
        //cout<<"layer "<<klayer<<" pose = "<<optimal_path_[klayer].transpose()<<endl;
        trip_cost += score_move(optimal_path_[klayer-1], optimal_path_[klayer]); // DEBUG/TEST
        //cout<<"trip cost so far = "<<trip_cost<<endl;
        
    }
    cout<<"recomputed trip cost = "<<trip_cost<<endl;
    cout<<"expected trip cost = "<<min_total_trip_cost<<endl;
    optimal_path = optimal_path_; // copy computed path to provided container
    return true;
}

//alt version that gets reference object passed to it for all path options:
//main routine: accept a large array of options, and return the optimal path
bool JointSpacePlanner::compute_optimal_path(vector<vector<Eigen::VectorXd> > &path_options) {
    //cout<<"computing all min costs"<<endl;
    //compute_all_min_costs();  // for this version, min costs already computed by constructor
    cout<<"finding optimal path"<<endl;
    double cost_to_go,min_total_trip_cost;
    double trip_cost; // debug

    // given the cost array, find the best path;
    // start by finding the optional starting node, layer 0
    int nstart_solns = all_costs_[0].size();
    int start_index=0;
    min_cost_to_go_=all_costs_[0][0];
    move_index_min_cost_to_go_=0;
    for (int istart=1;istart<nstart_solns;istart++) {
        cost_to_go = all_costs_[0][istart];
           if (cost_to_go< min_cost_to_go_) { // found a better option...
                min_cost_to_go_ = cost_to_go;
                move_index_min_cost_to_go_ = istart;
            }
    }
    trip_cost = 0;
    //cout<<"entire trip min cost is "<<min_cost_to_go_<<" starting from initial node "<<move_index_min_cost_to_go_<<endl;
    min_total_trip_cost_ =min_cost_to_go_;
    // fill up the path using optimal starting node and the identified best choices:
    optimal_path_[0] = path_options[0][move_index_min_cost_to_go_];
    // optimal_path_[0] = path_options[0][move_index_min_cost_to_go_];   // this does not work; must use member var
    //cout<<"layer 0 pose = "<<optimal_path_[0].transpose()<<endl;
    //cout<<"from layer 0, opt next node= "<<move_index_min_cost_to_go_<<endl;
    //klayer is next target layer:
    for (int klayer =1;klayer<nlayers_;klayer++) {
        //from node iopt in layer klayer-1, find the best move to layer k, per next_indices
        //from this index in klayer-1, find the corresponding pose and add it to optimal_path
        //step through the layers this way, from start to goal
        move_index_min_cost_to_go_= next_indices_[klayer-1][move_index_min_cost_to_go_];
        //cout<<"from layer "<<klayer-1<<", opt next node= "<<move_index_min_cost_to_go_<<endl;
        optimal_path_[klayer]= path_options[klayer][move_index_min_cost_to_go_];
        //cout<<"layer "<<klayer<<" pose = "<<optimal_path_[klayer].transpose()<<endl;
        trip_cost += score_move(optimal_path_[klayer-1], optimal_path_[klayer]); // DEBUG/TEST
        //cout<<"trip cost so far = "<<trip_cost<<endl;
        
    }
    cout<<"recomputed trip cost = "<<trip_cost<<endl;
    cout<<"expected trip cost = "<<min_total_trip_cost_<<endl;
    //solution is in member var optimal_path;
    // use a "get" method to return it to main
    //optimal_path = optimal_path_; // copy computed path to provided container
    return true;
}
