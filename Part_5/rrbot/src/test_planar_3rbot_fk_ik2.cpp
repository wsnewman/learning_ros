//test_3rbot_fk_ik: tests planar_3rbot_fk_ik library
// subscribes to joint values states;
// computes FK
// uses FK to compute IK
// then moves robot through sequence of solns redundant IK solns, keeping flange origin fixed

#include <rrbot/planar_3rbot_kinematics.h> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

Eigen::VectorXd g_q_vec;
using namespace std;

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    for (int i = 0; i < NJNTS; i++) {
        g_q_vec[i] = js_msg.position[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrbot_fk_test");

    std::vector<Eigen::VectorXd> jointspace_path;

    ros::NodeHandle nh;
    ros::Publisher pub1= nh.advertise<std_msgs::Float64>("/planar_3rbot/joint1_position_controller/command",1);
    ros::Publisher pub2= nh.advertise<std_msgs::Float64>("/planar_3rbot/joint2_position_controller/command",1);
    ros::Publisher pub3= nh.advertise<std_msgs::Float64>("/planar_3rbot/joint3_position_controller/command",1);
    Eigen::Vector3d q_init,q_prev;
    q_init<<0,0,0;
    g_q_vec= q_init; // init global q_vec

    ros::Subscriber joint_state_sub = nh.subscribe("planar_3rbot/joint_states", 1, jointStatesCb);
    //warm up joint states:
    g_q_vec[0]=100;
    while (g_q_vec[0]>99) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("received joint-state data; proceeding");
    jointspace_path.push_back(g_q_vec); // start motion from here;
    double q30 = g_q_vec[2];
    
    
    Planar_3rbot_fwd_solver planar_3rbot_fwd_solver;
    Planar_3rbot_IK_solver planar_3rbot_IK_solver;    
    Eigen::Vector3d flange_origin_wrt_world;
    Eigen::VectorXd computed_flange_origin, q_soln;
    Eigen::Affine3d affine_flange;
    affine_flange = planar_3rbot_fwd_solver.fwd_kin_flange_wrt_world_solve(g_q_vec);    
    flange_origin_wrt_world = affine_flange.translation(); // keep current flange origin as goal
    
    int n_solns=1;
    std::vector<Eigen::Vector3d> q_solns; //put IK solns here
    double dq1_sample_res = 0.01;
    int nsteps;
    double q1 = g_q_vec[0];
    while (n_solns>0) {
        q1+= dq1_sample_res;
        q_solns.clear();
        n_solns= planar_3rbot_IK_solver.solve_for_qsolns_given_q1(flange_origin_wrt_world, q1, q_solns);
        if (n_solns==1) {
            jointspace_path.push_back(q_solns[0]);
        }
        if (n_solns==2) {
            nsteps=jointspace_path.size();
            q_prev = jointspace_path[nsteps-1];
            double err0 = (q_prev - q_solns[0]).norm();
            double err1 = (q_prev - q_solns[1]).norm();
            if (err0<err1) jointspace_path.push_back(q_solns[0]);
            else jointspace_path.push_back(q_solns[1]);
        }
    }
    n_solns=1;
    q1-= dq1_sample_res;
    while (n_solns>0) {
        q1-= dq1_sample_res;
        q_solns.clear();
        n_solns= planar_3rbot_IK_solver.solve_for_qsolns_given_q1(flange_origin_wrt_world, q1, q_solns);
        if (n_solns==1) {
            jointspace_path.push_back(q_solns[0]);
        }
        if (n_solns==2) {
            nsteps=jointspace_path.size();
            q_prev = jointspace_path[nsteps-1];
            double err0 = (q_prev - q_solns[0]).norm();
            double err1 = (q_prev - q_solns[1]).norm();
            if (err0<err1) jointspace_path.push_back(q_solns[0]);
            else jointspace_path.push_back(q_solns[1]);
        }
    }
    
    //search q1 positive direction again, but choose q3 soln w/ opposite sign
    n_solns=1;
    q1+= dq1_sample_res;
    double q3;
    while (n_solns>0) {
        q1+= dq1_sample_res;
        q_solns.clear();
        n_solns= planar_3rbot_IK_solver.solve_for_qsolns_given_q1(flange_origin_wrt_world, q1, q_solns);
        q_soln = q_solns[0];
        q3 = q_soln[2];
        
        if (q30*q3<0) {
             jointspace_path.push_back(q_solns[0]);
        }
        else {
            jointspace_path.push_back(q_solns[1]);
        }
    }
    
    nsteps=jointspace_path.size();
    cout<<"computed null-space path: "<<endl;
    for (int istep=0;istep<nsteps;istep++){
        cout<<jointspace_path[istep].transpose()<<endl;
    }
    cout<<"test solns: Oflange from FK: "<<endl;    
    for (int istep=0;istep<nsteps;istep++){
        q_soln = jointspace_path[istep];
        affine_flange = planar_3rbot_fwd_solver.fwd_kin_flange_wrt_world_solve(q_soln);
        computed_flange_origin = affine_flange.translation();
        cout<<computed_flange_origin.transpose()<<endl;
    }      
    
    std_msgs::Float64 qmsg;
    for (int istep=0;istep<nsteps;istep++){
        q_soln = jointspace_path[istep];
        qmsg.data = q_soln[0];
        pub1.publish(qmsg);
        qmsg.data = q_soln[1];
        pub2.publish(qmsg);
        qmsg.data = q_soln[2];
        pub3.publish(qmsg); 
        ros::spinOnce();
        ros::Duration(0.05).sleep();        
    }     
    
}
