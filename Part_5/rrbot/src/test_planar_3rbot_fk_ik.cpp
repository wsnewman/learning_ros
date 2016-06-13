//test_3rbot_fk_ik: tests planar_3rbot_fk_ik library
// subscribes to joint values states;
// computes FK
// uses FK to compute IK
// compare IK solutions to actual answer

#include <rrbot/planar_3rbot_kinematics.h> 
#include <sensor_msgs/JointState.h>

Eigen::VectorXd g_q_vec;
using namespace std;

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    for (int i = 0; i < NJNTS; i++) {
        g_q_vec[i] = js_msg.position[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrbot_fk_test");

    ros::NodeHandle nh;
    Eigen::Vector3d q_init;
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
    Eigen::Affine3d affine_flange;

    Planar_3rbot_fwd_solver planar_3rbot_fwd_solver;
    Planar_3rbot_IK_solver planar_3rbot_IK_solver;    
    Eigen::Vector3d flange_origin_wrt_world;
    int n_solns;
    std::vector<Eigen::Vector3d> q_solns; //put IK solns here
    //double dq1_sample_res = 0.1;

   
    while (ros::ok()) {
        cout<<endl<<endl;
        ROS_INFO("angs: %f, %f, %f", g_q_vec[0], g_q_vec[1], g_q_vec[2]);

        affine_flange = planar_3rbot_fwd_solver.fwd_kin_flange_wrt_world_solve(g_q_vec);
        //for (int i = 0; i < NJNTS; i++) {
        //    cout << "frame " << i << " w/rt world: " << endl;
        //    cout << rrbot_fwd_solver.get_frame(i) << endl;
        //}

        flange_origin_wrt_world = affine_flange.translation();
        cout << "FK: flange origin: " << flange_origin_wrt_world.transpose() << endl;
        cout << "R_flange = " << endl;
        cout << affine_flange.linear() << endl;
        Eigen::Quaterniond quat(affine_flange.linear());
        cout<<"equiv quat: "<<quat.x()<<", "<< quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;

        
        ROS_INFO("computing inverse kinematics, given q1: "); 
        q_solns.clear();
        n_solns= planar_3rbot_IK_solver.solve_for_qsolns_given_q1(flange_origin_wrt_world, g_q_vec[0], q_solns);
        if (n_solns<1) cout << "found no viable IK solns" << endl;

        else {
            for (int i_soln=0;i_soln<n_solns;i_soln++) {
                cout<<"IK soln "<<i_soln<<": "<<q_solns[i_soln].transpose()<<endl;
            }
        }
        
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
}
