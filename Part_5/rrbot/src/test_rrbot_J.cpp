//test_rrbot_J.cpp
//uses random joint values and small joint perturbations to compare
// dp from FK vs dp = J*dq
#include <rrbot/rrbot_kinematics.h> 
#include <sensor_msgs/JointState.h>
#include <math.h>

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "rrbot_fk_test");
    ros::NodeHandle nh;

    Eigen::Affine3d affine_flange;

    Rrbot_fwd_solver rrbot_fwd_solver;
    //Rrbot_IK_solver rrbot_ik_solver;    
    
    Eigen::Vector3d flange_origin_wrt_world, perturbed_flange_origin, dp, Jdq_trans;
    Eigen::VectorXd Jdq;

    double q_elbow, q_shoulder; 
    double q_elbow_perturbed,q_shoulder_perturbed;
    double delta_q = 0.000001;
    Eigen::MatrixXd J;
    double diff;
    Eigen::MatrixXd q_vec(2,1),dq_vec(2,1);
    q_vec<<0,0;
    dq_vec<<delta_q,delta_q;

    while (ros::ok()) {
        cout<<endl<<endl;

        q_vec(0) = q_shoulder;
        q_vec(1) = q_elbow;
        ROS_INFO("angs: %f, %f", q_elbow, q_shoulder);

        affine_flange = rrbot_fwd_solver.fwd_kin_flange_wrt_world_solve(q_vec);
        flange_origin_wrt_world = affine_flange.translation();
        cout << "FK: flange origin: " << flange_origin_wrt_world.transpose() << endl;
        //perturb the joint angles and recompute fwd kin:
        affine_flange = rrbot_fwd_solver.fwd_kin_flange_wrt_world_solve(q_vec+dq_vec);
        perturbed_flange_origin = affine_flange.translation();
        dp = perturbed_flange_origin - flange_origin_wrt_world;
        cout<<"dp = "<<dp.transpose()<<endl;
        J = rrbot_fwd_solver.Jacobian(q_vec);
        cout<<"J: "<<endl;
        cout<<J<<endl;
        cout<<"dq = "<<dq_vec.transpose()<<endl;
        Jdq = J*dq_vec;
        cout<<"Jdq = "<<Jdq.transpose()<<endl;
        
        for (int i=0;i<3;i++) Jdq_trans(i) = Jdq(i);

        cout<<"Jdq_trans = "<<Jdq_trans.transpose()<<endl;
        diff = (Jdq_trans - dp).norm();
        cout<<"diff = "<<diff<<endl;
        
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        q_elbow = q_lower_limits[0] + (q_upper_limits[0]-q_lower_limits[0])*(rand() % 100)/100.0;  
        q_shoulder = q_lower_limits[0] + (q_upper_limits[1]-q_lower_limits[1])*(rand() % 100)/100.0;        
    }
}
