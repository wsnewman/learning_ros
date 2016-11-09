// library of useful functions for controlling Arm7dof arm motions
#include <arm7dof_traj_as/arm7dof_traj_as.h>


Arm7dof_traj_streamer::Arm7dof_traj_streamer(ros::NodeHandle* nodehandle){
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();     
  
    qdot_max_vec_<<q0dotmax,q1dotmax,q2dotmax,q3dotmax,q4dotmax,q5dotmax,q6dotmax;
    qdot_max_vec_ *=SPEED_SCALE_FACTOR;

    
    q_vec_Xd_.resize(7);
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void Arm7dof_traj_streamer::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("arm7dof/joint_states", 1, &Arm7dof_traj_streamer::jointStatesCb,this);  
}

//member helper function to set up publishers;
void Arm7dof_traj_streamer::initializePublishers()
{
    ROS_INFO("Initializing Publishers");

    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("trajActionServer", 1); 
}



//NOTE: this is not separately threaded.  this callback only responds when the parent node allows a ros spin.
// subscribe to joint angles; copy over the joint angles into member var
//parse the names in joint_names vector; find the corresponding indices for the joints of interest
void Arm7dof_traj_streamer::map_arm_joint_indices(vector<string> joint_names) {
 //vector<string> joint_names = joint_state->name;
    //vector<string> jnt_names;
    joint_indices_.clear();

    int index;
    int n_jnts = joint_names.size();
    cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

   for (int j=0;j<arm7dof_NJNTS;j++) {
       j_name = g_arm7dof_jnt_names[j];  
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            joint_indices_.push_back(index);
            break;
        }
       }
      
   }   
   cout<<"indices of arm joints: "<<endl;
   for (int i=0;i<arm7dof_NJNTS;i++) {
       cout<<joint_indices_[i]<<", ";
   }
}

void Arm7dof_traj_streamer::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    joint_states_ = js_msg; // copy this to member var
    map_arm_joint_indices(js_msg.name);
       for (int i=0;i<arm7dof_NJNTS;i++)
       {
        q_vec_[i] = js_msg.position[joint_indices_[i]];   
        q_vec_Xd_[i] = q_vec_[i]; //alt data type: Eigen::VectorXd
        }   
}  

//compute arrival time based on slowest joint arrival time 
double Arm7dof_traj_streamer::transition_time(Vectorq7x1 dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec_[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec_[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

//same, but w/ VectorXd arg
double Arm7dof_traj_streamer::transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec_[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec_[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

//convert a path to a trajectory; add joint names, timing, and put in trajectory_msg
void Arm7dof_traj_streamer::stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("joint0");
    new_trajectory.joint_names.push_back("joint1");
    new_trajectory.joint_names.push_back("joint2");
    new_trajectory.joint_names.push_back("joint3");
    new_trajectory.joint_names.push_back("joint4");
    new_trajectory.joint_names.push_back("joint5");
    new_trajectory.joint_names.push_back("joint6");

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
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }   
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }        
        
}    


//publish a trajectory; to be processed by a separate interpolating joint commander
void Arm7dof_traj_streamer::pub_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory) {
    traj_pub_.publish(new_trajectory);
    cout<<"publishing arm trajectory with npts = "<<new_trajectory.points.size()<<endl;
}


// get current arm joint angles, and send some trajectories with these angles, just to get listener warmed up
void Arm7dof_traj_streamer::pub_arm_trajectory_init() {
    std::vector<Eigen::VectorXd> qvec;
    trajectory_msgs::JointTrajectory new_trajectory;
    //create a dummy trajectory, consisting of current arm poses
    qvec.push_back(q_vec_Xd_);
    qvec.push_back(q_vec_Xd_);   
    stuff_trajectory(qvec, new_trajectory);  
    traj_pub_.publish(new_trajectory);
}
