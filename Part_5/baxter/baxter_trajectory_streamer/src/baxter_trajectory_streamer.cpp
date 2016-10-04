// library of useful functions for controlling Baxter motions
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>


Baxter_traj_streamer::Baxter_traj_streamer(ros::NodeHandle* nodehandle){
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();    
  left_cmd_.mode = 1; // set the command modes to "position"
  right_cmd_.mode = 1;

// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
  right_cmd_.names.push_back("right_s0");
  right_cmd_.names.push_back("right_s1");
  right_cmd_.names.push_back("right_e0");
  right_cmd_.names.push_back("right_e1");
  right_cmd_.names.push_back("right_w0");
  right_cmd_.names.push_back("right_w1");
  right_cmd_.names.push_back("right_w2");
// same order for left arm
  left_cmd_.names.push_back("left_s0");
  left_cmd_.names.push_back("left_s1");
  left_cmd_.names.push_back("left_e0");
  left_cmd_.names.push_back("left_e1");
  left_cmd_.names.push_back("left_w0");
  left_cmd_.names.push_back("left_w1");
  left_cmd_.names.push_back("left_w2");

  // do push-backs to establish desired vector size with valid joint angles
  for (int i=0;i<7;i++) {
     right_cmd_.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd_.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   }    
  
    qdot_max_vec_<<q0dotmax,q1dotmax,q2dotmax,q3dotmax,q4dotmax,q5dotmax,q6dotmax;
    qdot_max_vec_ *=SPEED_SCALE_FACTOR;
    
    q_vec_right_arm_Xd_.resize(7);
    q_vec_left_arm_Xd_.resize(7);

}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void Baxter_traj_streamer::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("robot/joint_states", 1, &Baxter_traj_streamer::jointStatesCb,this);  
}

//member helper function to set up publishers;
void Baxter_traj_streamer::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    joint_cmd_pub_right_ = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, true); 
    joint_cmd_pub_left_  = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1, true);
    right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1); 
    left_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("left_arm_joint_path_command", 1); 
}

//parse the names in joint_names vector; find the corresponding indices for left and right arm joints
void Baxter_traj_streamer::map_arms_joint_indices(vector<string> joint_names) {
 //vector<string> joint_names = joint_state->name;
    vector<string> rt_limb_jnt_names;
        
    right_arm_joint_indices_.clear();
    left_arm_joint_indices_.clear();
    int index;
    int n_jnts = joint_names.size();
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

   for (int j=0;j<7;j++) {
       j_name = right_cmd_.names[j];  
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            right_arm_joint_indices_.push_back(index);
            break;
        }
       }
       j_name = left_cmd_.names[j];  
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            left_arm_joint_indices_.push_back(index);
            break;
        }        
       }
   }   
   //cout<<"indices of right-arm joints: "<<endl;
   //for (int i=0;i<7;i++) {
       //cout<<right_arm_joint_indices_[i]<<", ";
   //}
   //cout<<endl;
   //cout<<"indices of left-arm joints: "<<endl;
   //for (int i=0;i<7;i++) {
    //   cout<<left_arm_joint_indices_[i]<<", ";
   //}
   //cout<<endl;   
}


//NOTE: this is not separately threaded.  this callback only responds with the parent node allows a ros spin.
// subscribe to joint angles; parse the message to populate ordered vectors of jnt angles, left and right
void Baxter_traj_streamer::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    joint_states_ = js_msg; // copy this to member var
    if (right_arm_joint_indices_.size()<1) {
       //g_all_jnt_names = js_msg.name;
       map_arms_joint_indices(js_msg.name);
       for (int i=0;i<7;i++)
       {
        q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices_[i]]; //w2    
        q_vec_left_arm_[i] = js_msg.position[left_arm_joint_indices_[i]];
        q_vec_right_arm_Xd_[i] = q_vec_right_arm_[i]; //alt data type: Eigen::VectorXd
        q_vec_left_arm_Xd_[i] = q_vec_left_arm_[i];
        }
        cout<<"CB: q_vec_right_arm: "<<q_vec_right_arm_Xd_.transpose()<<endl;       
       
    }
    // copy right-arm angles to global vec
    for (int i=0;i<7;i++)
    {
        q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices_[i]]; //w2    
        q_vec_left_arm_[i] = js_msg.position[left_arm_joint_indices_[i]];
        q_vec_right_arm_Xd_[i] = q_vec_right_arm_[i]; //alt data type: Eigen::VectorXd
        q_vec_left_arm_Xd_[i] = q_vec_left_arm_[i];
    }
    //cout<<"CB: q_vec_right_arm: "<<q_vec_right_arm_.transpose()<<endl;
    
}  

Vectorq7x1 Baxter_traj_streamer::get_qvec_right_arm() {
    return q_vec_right_arm_;
}

Vectorq7x1 Baxter_traj_streamer::get_qvec_left_arm() {
    return q_vec_left_arm_;
}

//re-use this for both left and right; same size, same vel limits, 
double Baxter_traj_streamer::transition_time(Vectorq7x1 dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec_[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec_[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

double Baxter_traj_streamer::transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec_[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec_[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}


void Baxter_traj_streamer::stuff_trajectory_right_arm( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];   
     //cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];
 
    trajectory_point1.time_from_start =    ros::Duration(net_time); 
    for (int i=0;i<7;i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    } 
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs
   

    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time+= del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }   
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }        
        
}    

void Baxter_traj_streamer::stuff_trajectory_left_arm( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("left_s0");
    new_trajectory.joint_names.push_back("left_s1");
    new_trajectory.joint_names.push_back("left_e0");
    new_trajectory.joint_names.push_back("left_e1");
    new_trajectory.joint_names.push_back("left_w0");
    new_trajectory.joint_names.push_back("left_w1");
    new_trajectory.joint_names.push_back("left_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];   
     //cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 

    //trajectory_point1.positions = qvecs[0];
 
    trajectory_point1.time_from_start =    ros::Duration(net_time); 
    for (int i=0;i<7;i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    } 
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs
   

    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }   
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }        
        
}    

// command a single pose: cmd_pose_right(Vectorq7x1 qvec );
void Baxter_traj_streamer::cmd_pose_right(Vectorq7x1 qvec ) {
    //member var right_cmd_ already has joint names populated
    for (int i=0;i<7;i++) {
        right_cmd_.command[i]=qvec[i];
    }
    joint_cmd_pub_right_.publish(right_cmd_);
}

//publish a trajectory; to be processed by a separate interpolating joint commander
void Baxter_traj_streamer::pub_right_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory) {
    right_traj_pub_.publish(new_trajectory);
    cout<<"publishing right-arm trajectory with npts = "<<new_trajectory.points.size()<<endl;
}

void Baxter_traj_streamer::pub_left_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory) {
    left_traj_pub_.publish(new_trajectory);
    cout<<"publishing left-arm trajectory with npts = "<<new_trajectory.points.size()<<endl;
}


// get current arm joint angles, and send some trajectories with these angles, just to get listener warmed up
void Baxter_traj_streamer::pub_arms_trajectory_init() {
    std::vector<Eigen::VectorXd> qvecs_right,qvecs_left;
    trajectory_msgs::JointTrajectory new_trajectory_right,new_trajectory_left;
    //create a dummy trajectory, consisting of current arm poses
    qvecs_right.push_back(q_vec_right_arm_Xd_);
    qvecs_right.push_back(q_vec_right_arm_Xd_);  
    qvecs_left.push_back(q_vec_left_arm_Xd_);
    qvecs_left.push_back(q_vec_left_arm_Xd_);      
    stuff_trajectory_right_arm(qvecs_right, new_trajectory_right);  
    stuff_trajectory_left_arm(qvecs_left, new_trajectory_left);
    right_traj_pub_.publish(new_trajectory_right);
    left_traj_pub_.publish(new_trajectory_left);
}