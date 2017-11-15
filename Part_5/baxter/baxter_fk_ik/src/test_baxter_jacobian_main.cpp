// test_baxter_jacobian_main.cpp
// wsn, Nov 2017
//compare Jacobian to differential forward kinematics

//11/8/17: dp looks good, comparing fwd_kin(q+dq)-fwd_kin(q) to
// J*dq
// HOWEVER, orientation change looks questionable.  Needs more testing/debugging


#include <baxter_fk_ik/baxter_kinematics.h> 
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <fstream>
using namespace std;

//some globals:
    Vectorq7x1 q_vec_right_arm_; //,q_in,q_soln,q_snapshot; 
    Vectorq7x1 q_vec_left_arm_;
    Eigen::VectorXd q_vec_right_arm_Xd_,q_vec_left_arm_Xd_;
    baxter_core_msgs::JointCommand right_cmd,left_cmd;
    vector<int> right_arm_joint_indices_;    
    vector<int> left_arm_joint_indices_;  
    sensor_msgs::JointState joint_states_;

void   do_some_inits() {
    right_cmd.names.push_back("right_s0");
    right_cmd.names.push_back("right_s1");
    right_cmd.names.push_back("right_e0");
    right_cmd.names.push_back("right_e1");
    right_cmd.names.push_back("right_w0");
    right_cmd.names.push_back("right_w1");
    right_cmd.names.push_back("right_w2");
    left_cmd.names.push_back("left_s0");
    left_cmd.names.push_back("left_s1");
    left_cmd.names.push_back("left_e0");
    left_cmd.names.push_back("left_e1");
    left_cmd.names.push_back("left_w0");
    left_cmd.names.push_back("left_w1");
    left_cmd.names.push_back("left_w2");    
    
    q_vec_right_arm_Xd_.resize(7); //populate from jointStatesCb callback
    q_vec_left_arm_Xd_.resize(7);    
}
    
    
void map_arms_joint_indices(vector<string> joint_names) {
 //vector<string> joint_names = joint_state->name;
    vector<string> rt_limb_jnt_names;
        
    right_arm_joint_indices_.clear();
    left_arm_joint_indices_.clear();
    int index;
    int n_jnts = joint_names.size();
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

   for (int j=0;j<7;j++) {
       j_name = right_cmd.names[j];  
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            right_arm_joint_indices_.push_back(index);
            break;
        }
       }
       j_name = left_cmd.names[j];  
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            left_arm_joint_indices_.push_back(index);
            break;
        }        
       }
   }     
}


void jointStatesCb(const sensor_msgs::JointState& js_msg) {
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


int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_jacobian_test");
    ros::NodeHandle n;
    ros::Subscriber joint_state_sub= n.subscribe("robot/joint_states",1,jointStatesCb); 
  
    Baxter_fwd_solver baxter_fwd_solver;
    Eigen::Vector3d p1,p2,dp;
    Eigen::Vector3d n_des,t_des,b_des;
    Eigen::MatrixXd Jacobian;
    Vectorq7x1 q_in1,q_in2,dq;
    Eigen::VectorXd q_vec_xd,dq_vec_xd;
    Eigen::Affine3d fwd_kin_tool_wrt_torso;   
    Eigen::Matrix3d Ra,Rb,dR;
    do_some_inits();
    
    

    Eigen::VectorXd dpose_6x1;
    dpose_6x1.resize(6);
    q_vec_xd.resize(7);

    q_in1<<0,0,0,0,0,0,0;
    q_vec_xd = q_in1;
    dq<<0,1,0,0,0,0,0;
    double eps = 0.000001;
    dq= dq*eps;
    dq_vec_xd = dq;
    q_in2= q_in1+dq;    
    q_vec_right_arm_ = q_in1; //default, if not running simu in Gazebo
    
    for (int i=0;i<10;i++) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    
    Eigen::Matrix4d A4x4a,A4x4b,dA;
    Eigen::Matrix4d  A_torso_to_rarm_mount,A4x4_tool_wrt_flange,A_rarm_mount_to_r_lower_forearm;   
    A_torso_to_rarm_mount = baxter_fwd_solver.get_A_torso_to_rarm_mount();// { return  A_torso_to_rarm_mount_;}    
    A4x4_tool_wrt_flange=baxter_fwd_solver.get_A4x4_tool_wrt_flange();
    //A_rarm_mount_to_r_lower_forearm = baxter_fwd_solver.get_A4x4_rarm_mount_to_r_lower_forearm();
    //A_rarm_mount_to_r_lower_forearm_ is already taken into account in fwd_kin_solve_()
  while(ros::ok()) { 
    ros::spinOnce(); //update joint sensors readings
    ros::Duration(0.1).sleep();
    ros::spinOnce();  
    
    A4x4a = A_torso_to_rarm_mount*(baxter_fwd_solver.fwd_kin_solve_(q_vec_right_arm_))*A4x4_tool_wrt_flange;
    //cout<<"A4x4: "<<endl;
    //cout<<A4x4a<<endl;
    Eigen::Vector3d tool_origin = A4x4a.block<3,1>(0,3);

    Jacobian = baxter_fwd_solver.compute_Jacobian(q_vec_right_arm_);
    
    A4x4b = A_torso_to_rarm_mount*(baxter_fwd_solver.fwd_kin_solve_(q_vec_right_arm_+dq))*A4x4_tool_wrt_flange;

    Ra = A4x4a.block<3,3>(0,0);
    Rb = A4x4b.block<3,3>(0,0);  
    dR = Ra.inverse()*Rb;
    Eigen::Matrix3d R_test;
    R_test = Ra*Ra.transpose();
    cout<<"R_test for Ra = "<<endl;
    cout<<R_test<<endl;
    R_test = Rb*Rb.transpose();
    cout<<"R_test for Rb = "<<endl;
    cout<<R_test<<endl;    


    //dA = A4x4b-A4x4a;
    //dp = dA.block<3,1>(0,3);
    
    //dpose_6x1 = Jacobian*dq_vec_xd;
    //cout<<"dpose: "<<dpose_6x1.transpose()<<endl;
    //cout<<"dA: "<<endl;
    //cout<<dA<<endl;
  
    cout<<"right arm angles: "<<q_vec_right_arm_.transpose()<<endl;
    cout<<"dq: "<<dq.transpose()<<endl;
    //compute fwd kin using fk_ik functions:
    fwd_kin_tool_wrt_torso=baxter_fwd_solver.fwd_kin_tool_wrt_torso_solve(q_vec_right_arm_);

    p1 = fwd_kin_tool_wrt_torso.translation();
    Ra = fwd_kin_tool_wrt_torso.linear();
    cout<<"fwd_solve origin: "<<p1.transpose()<<endl;
    //this alternative was computed within this fnc, using fwd_kin_solve_ and rarm and tool transforms
    cout<<"alt tool origin:  "<<tool_origin.transpose()<<endl;
    //above looks good, --> use of transforms is correct
    
    //use fk_ik fncs to compute pose of perturbed arm:
    fwd_kin_tool_wrt_torso=baxter_fwd_solver.fwd_kin_tool_wrt_torso_solve(q_vec_right_arm_+dq);
    
    p2 = fwd_kin_tool_wrt_torso.translation();
    Rb = fwd_kin_tool_wrt_torso.linear();
 
    dR = Rb*Ra.transpose();   
    
    //want to explain rotation Ra to Rb:
    // R_change*Ra = Rb
    // look at Rb*Ra_inv 
    
    dp = p2-p1; //Cartesian perturbation of tool origin, per fk_ik funcs
    cout<<"dp fwd kin = "<<dp.transpose()<<endl;
    //compare to tool-frame perturbation as derived using Jacobian:
    dpose_6x1 = Jacobian*dq_vec_xd;
    cout<<"dpose per J: "<<dpose_6x1.transpose()<<endl;  
    cout<<"dR from fwd kin orientations: "<<endl;
    cout<<dR<<endl;        
    cout<<"R2-R1 from fwd kin orientations: "<<endl;
    dR = Rb-Ra;
    cout<<dR<<endl; 
    cout<<"Ra: "<<endl;
    cout<<Ra<<endl;
        cout<<"Rb: "<<endl;
    cout<<Rb<<endl;
    ros::Duration(2).sleep();
    }
 
    return 0;
}
