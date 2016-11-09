//NAC controller: uses force sensor values and virtual attractors to 
// derive corresponding joint velocities, which are commanded
// to velocity controllers

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <arm7dof_fk_ik/arm7dof_kinematics.h>

using namespace std;

const double q0_dot_max = 0.17;
const double q1_dot_max = 0.17;
const double q2_dot_max = 0.3;
const double q3_dot_max = 0.3;
const double q4_dot_max = 0.4;
const double q5_dot_max = 0.4;
const double q6_dot_max = 0.4;

Eigen::VectorXd g_q_des; //want to get this via a topic/callback
Eigen::VectorXd g_q_vec_actual, g_qdot_vec_actual;
Vectorq7x1 g_q_actual_7x1;
//Eigen::VectorXd g_qdot_max_vec;
Eigen::VectorXd J_transpose_f;
Eigen::Vector3d g_f_sensor;
double g_force_z=0.0;
Eigen::VectorXd g_wrench;

ros::Publisher    g_j0_pub;
ros::Publisher    g_j1_pub;
ros::Publisher    g_j2_pub;
ros::Publisher    g_j3_pub;
ros::Publisher    g_j4_pub;
ros::Publisher    g_j5_pub;
ros::Publisher    g_j6_pub;


void qDesCB(const std_msgs::Float64MultiArray q_des_msg) 
{ 
    for (int i=0;i<7;i++) {
       g_q_des[i] =  q_des_msg.data[i]; 
    }
    cout<<"qDesCB: "<<g_q_des.transpose()<<endl;
} 


void sat_qdot(Eigen::VectorXd &q_dot_cmd_vec){
    for (int i=0;i<7;i++) {
        q_dot_cmd_vec[i] = std::min(q_dot_cmd_vec[i],g_qdot_max_vec[i]);
        q_dot_cmd_vec[i] = std::max(q_dot_cmd_vec[i],-g_qdot_max_vec[i]);
    }
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
       for (int i=0;i<7;i++)
       {
        g_q_vec_actual[i] = js_msg.position[i+1]; // first jnt is force sensor; SHOULD match jnt names
        g_qdot_vec_actual[i] = js_msg.velocity[i+1];
        g_q_actual_7x1[i] = g_q_vec_actual[i]; // alt datatype
        }   
}  

//subscribe to ft_sensor_topic w/ type geometry_msgs/WrenchStamped;
// extract component wrench.force.z
void ft_sensor_CB(const geometry_msgs::WrenchStamped& ft) 
{ 
  g_force_z = ft.wrench.force.z;
  //really, want to express this in base frame...
  g_f_sensor[0] = ft.wrench.force.x;
  g_f_sensor[1] = ft.wrench.force.y;
  g_f_sensor[2] = ft.wrench.force.z;
} 

void send_qdot_cmds(Eigen::VectorXd qdot_cmd_vec) {
    std_msgs::Float64 cmd_msg;
    cmd_msg.data = qdot_cmd_vec[0];
    g_j0_pub.publish(cmd_msg);
    
    cmd_msg.data = qdot_cmd_vec[1];
    g_j1_pub.publish(cmd_msg);

    cmd_msg.data = qdot_cmd_vec[2];
    g_j2_pub.publish(cmd_msg);

    cmd_msg.data = qdot_cmd_vec[3];
    g_j3_pub.publish(cmd_msg);

    cmd_msg.data = qdot_cmd_vec[4];
    g_j4_pub.publish(cmd_msg);

    cmd_msg.data = qdot_cmd_vec[5];
    g_j5_pub.publish(cmd_msg);

    cmd_msg.data = qdot_cmd_vec[6];
    g_j6_pub.publish(cmd_msg);    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "NAC_control_loop"); 
    ros::NodeHandle nh; 
    Arm7dof_IK_solver arm7dof_ik_solver;
    Arm7dof_fwd_solver arm7dof_fwd_solver;
    
    Eigen::VectorXd q_ddot_vec,q_dot_cmd_vec;
    Eigen::VectorXd q_vec_err= Eigen::VectorXd::Zero(7,1);
    Eigen::MatrixXd Kq_on_H = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd H_inv = Eigen::MatrixXd::Zero(7,7);    
    Eigen::MatrixXd B_on_H= Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd B_jnts= Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd I7x7 = Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd wrench_6x1= Eigen::VectorXd::Zero(6,1);
    Eigen::MatrixXd Jacobian;
    Eigen::MatrixXd J_dx_dq= Eigen::MatrixXd::Zero(3,7);
    Eigen::Vector3d endpt_wrt_base, endpt_attractor, f_attractor, f_net, v_cartesian;
    Eigen::Affine3d affine_flange; 
    Eigen::Matrix3d K_cartesian = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d B_cartesian = Eigen::Matrix3d::Identity(3,3);
    
    Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity(3,3); //specify desired tool-flange orientation
    
    //set gains here, presumably based on H and K_des (desired compliances)
    cout<<"set Kq_on_H vals..."<<endl;
    Kq_on_H(0,0) = 1.0;
    cout<<"more vals..."<<endl;
    
    //estimate the natural inertias; diagonal components only
    H_inv(0,0) = 1/20.0;
    H_inv(1,1) = 1/20.0;
    H_inv(2,2) = 1/10.0;
    H_inv(3,3) = 1/10.0;
    H_inv(4,4) = 1/3.0;
    B_jnts= 10*B_jnts; // joint-space dampers
    B_jnts(1,1) = 100;
    B_jnts(2,2) = 100;
    /*
    Kq_on_H(1,1) = 1.0;
    Kq_on_H(2,2) = 1.0;
    Kq_on_H(3,3) = 1.0;
    Kq_on_H(4,4) = 1.0;
    Kq_on_H(5,5) = 1.0;
    Kq_on_H(6,6) = 1.0;*/
    Kq_on_H = 1.0*I7x7;
    //Kq_on_H(6,6) = 1.0;
    //Kq_on_H = 10.0*Kq_on_H;
    B_on_H = 1.0*I7x7;
    //B_on_H(6,6) = 2.0;
    //set max vel values; better would be to read these from parameter server
    /*
    cout<<"g_qdot_max_vec..."<<endl;
    g_qdot_max_vec= q_vec_err;
    cout<<"assign vel limits..."<<endl;
    g_qdot_max_vec[0] = q0_dot_max;
    g_qdot_max_vec[1] = q1_dot_max;
    g_qdot_max_vec[2] = q2_dot_max;
    g_qdot_max_vec[3] = q3_dot_max;
    g_qdot_max_vec[4] = q4_dot_max;
    g_qdot_max_vec[5] = q5_dot_max;
    g_qdot_max_vec[6] = q6_dot_max;
    cout<<"so far, so good..."<<endl;
    */
    double dt = 0.001; // 1000Hz loop rate desired
    ros::Rate naptime(1/dt);    
    
    g_q_des=q_vec_err;
    for (int i=0;i<7;i++) g_q_des[i]=0.0;
    //init desirable pose:
    g_q_des[1] = 1.0;
    g_q_des[3] = -2.0;
    g_q_des[5] = 1.0;
    
    //set cartesian attractor:
    endpt_attractor<<0,0,1.5;
    g_q_vec_actual=g_q_des;
    g_qdot_vec_actual=q_vec_err;
    
    //set Cartesian spring stiffnesses:
    K_cartesian(0,0) = 100.0;
    K_cartesian(1,1) = 100.0;
    K_cartesian(2,2) = 10.0;
    B_cartesian = 10*B_cartesian;
    const double K_vel_p = 20.0; //command velocity proportional to ang error; K_vel_p = bw (rad/sec)
    
    q_ddot_vec=q_vec_err;
    q_dot_cmd_vec=q_vec_err;
    J_transpose_f = q_vec_err;
    q_vec_err = g_q_des-g_q_vec_actual;    
    
    g_wrench = wrench_6x1;

    //set up joint velocity-command publishers
    g_j0_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint0_velocity_controller/command", 1); 
    g_j1_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint1_velocity_controller/command", 1); 
    g_j2_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint2_velocity_controller/command", 1); 
    g_j3_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint3_velocity_controller/command", 1); 
    g_j4_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint4_velocity_controller/command", 1); 
    g_j5_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint5_velocity_controller/command", 1); 
    g_j6_pub =  nh.advertise<std_msgs::Float64>("/arm7dof/joint6_velocity_controller/command", 1); 
    
    //subscribe to the joint states;
    ros::Subscriber joint_state_sub = nh.subscribe("arm7dof/joint_states", 1, jointStatesCb);  
    ros::Subscriber q_des_sub = nh.subscribe("qdes_attractor_vec", 1, qDesCB); 
    ros::Subscriber ft_sensor_sub = nh.subscribe("ft_sensor_topic", 1, ft_sensor_CB);
    cout<<"starting loop..."<<endl;
    bool is_singular=false;
    std::vector<Vectorq7x1> q_solns;
    Vectorq7x1 q_soln, q_vec_err_wrist;
    while (ros::ok()) 
    {
        ros::spinOnce();
        //do interesting computations here: vel cmd based on q_des and q_actual
        //cout<<"g_q_vec_actual: "<<g_q_vec_actual.transpose()<<endl;
        Jacobian = arm7dof_fwd_solver.Jacobian(g_q_vec_actual);
        J_dx_dq = Jacobian.block<3,7>(0,0);
        affine_flange = arm7dof_fwd_solver.fwd_kin_flange_wrt_base_solve(g_q_vec_actual);
        
        endpt_wrt_base = affine_flange.translation();  
        cout<<"endpt_wrt_base: "<<endpt_wrt_base.transpose()<<endl;  
        cout<<"g_f_sensor: "<<g_f_sensor.transpose()<<endl;
        v_cartesian =  J_dx_dq*g_qdot_vec_actual;
        f_attractor = K_cartesian*(endpt_attractor-endpt_wrt_base) - B_cartesian*v_cartesian;
        cout<<"f_attractor: "<<f_attractor.transpose()<<endl;
        f_net = f_attractor + g_f_sensor;
        cout<<"f_net: "<<f_net.transpose()<<endl;
        J_transpose_f = J_dx_dq.transpose()*f_net; //equiv jnt trqs
        
        q_ddot_vec = H_inv*(J_transpose_f - B_on_H*g_qdot_vec_actual);
           
        //q_vec_err = g_q_des-g_q_vec_actual;
        //cout<<"q_vec_err: "<<q_vec_err.transpose()<<endl;
        //q_ddot_vec = 0.1*q_vec_err-g_qdot_vec_actual; //watch out--need to include influence of endpoint forces, else will ramp to saturation
        //q_ddot_vec = Kq_on_H*q_vec_err -  B_on_H*g_qdot_vec_actual;
        q_dot_cmd_vec+= q_ddot_vec*dt; //integrate accel eqns to get desired qdot vals
        
        //treat the wrist DOF's separately: servo these to maintain desired flange orientation, R_des
//bool solve_spherical_wrist(Vectorq7x1 q_in,Eigen::Matrix3d R_des, std::vector<Vectorq7x1> &q_solns);
        is_singular = arm7dof_ik_solver.solve_spherical_wrist(g_q_actual_7x1, R_des, q_solns);
        cout<<"there are " <<q_solns.size()<<" wrist solns"<<endl;
        if (q_solns.size()>0) {
           q_vec_err_wrist = q_solns[0]-g_q_actual_7x1;
            if (q_solns.size()>1) {         
                double q_err1 = q_vec_err_wrist.norm();
                double q_err2 = (q_solns[1] - g_q_actual_7x1).norm();
                if (q_err2<q_err1) q_vec_err_wrist = q_solns[1] - g_q_actual_7x1;
            }
            cout<<"wrist ang err: "<<q_vec_err_wrist.transpose()<<endl;
            for (int i=4; i<7; i++) {
              q_dot_cmd_vec[i] = K_vel_p*q_vec_err_wrist[i];
            }
         }
        
        
        sat_qdot(q_dot_cmd_vec); //anti-windup based on qdot_max for each jnt;//Kq_on_H*q_vec_err; // - B_on_H*g_qdot_vec_actual; //watch out--need to include influence of endpoint forces, else will ramp to saturation
        q_dot_cmd_vec+= q_ddot_vec*dt; //integrate accel eqns to get desired qdot vals
        sat_qdot(q_dot_cmd_vec); //anti-windup based on qdot_max for each jnt
        send_qdot_cmds(q_dot_cmd_vec); //command these velocities
        //cout<<"qdot_cmd_vec: "<<q_dot_cmd_vec.transpose()<<endl;
        ros::spinOnce();
        naptime.sleep(); 
    }
}

