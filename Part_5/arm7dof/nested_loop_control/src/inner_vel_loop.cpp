//inner_vel_loop:  
// given q_des, command joint velocities:


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
Eigen::VectorXd g_qdot_max_vec;

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
        }   
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
    ros::init(argc, argv, "inner_vel_loop"); 
    ros::NodeHandle nh; 
    
    Eigen::VectorXd q_ddot_vec,q_dot_cmd_vec;
    Eigen::VectorXd q_vec_err= Eigen::VectorXd::Zero(7,1);
    Eigen::MatrixXd Kq_on_H = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd B_on_H= Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd I7x7 = Eigen::MatrixXd::Identity(7,7);

    //set gains here, presumably based on H and K_des (desired compliances)
    cout<<"set Kq_on_H vals..."<<endl;
    //Kq_on_H(0,0) = 1.0;
    //cout<<"more vals..."<<endl;
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
    //cout<<"so far, so good..."<<endl;
    
    double dt = 0.001; // 1000Hz loop rate desired
    ros::Rate naptime(1/dt);    
    
    g_q_des=q_vec_err;
    for (int i=0;i<7;i++) g_q_des[i]=0.0;
    //init desirable pose:
    g_q_des[1] = 1.0;
    g_q_des[3] = -2.0;
    g_q_des[5] = 1.0;
    g_q_vec_actual=g_q_des;
    g_qdot_vec_actual=q_vec_err;
    
    q_ddot_vec=q_vec_err;
    q_dot_cmd_vec=q_vec_err;
    q_vec_err = g_q_des-g_q_vec_actual;    

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
    cout<<"starting position control loop using inner velocity control loop."<<endl;
    cout<<"Listening for desired joint values on topic qdes_attractor_vec."<<endl;    
    while (ros::ok()) 
    {
        ros::spinOnce();
        //do interesting computations here: vel cmd based on q_des and q_actual
        //cout<<"g_q_vec_actual: "<<g_q_vec_actual.transpose()<<endl;
        q_vec_err = g_q_des-g_q_vec_actual;
        //cout<<"q_vec_err: "<<q_vec_err.transpose()<<endl;
        //q_ddot_vec = 0.1*q_vec_err-g_qdot_vec_actual; //watch out--need to include influence of endpoint forces, else will ramp to saturation
        q_ddot_vec = Kq_on_H*q_vec_err -  B_on_H*g_qdot_vec_actual;
        q_dot_cmd_vec+= q_ddot_vec*dt; //integrate accel eqns to get desired qdot vals
        sat_qdot(q_dot_cmd_vec); //anti-windup based on qdot_max for each jnt;//Kq_on_H*q_vec_err; // - B_on_H*g_qdot_vec_actual; //watch out--need to include influence of endpoint forces, else will ramp to saturation
        q_dot_cmd_vec+= q_ddot_vec*dt; //integrate accel eqns to get desired qdot vals
        sat_qdot(q_dot_cmd_vec); //anti-windup based on qdot_max for each jnt
        send_qdot_cmds(q_dot_cmd_vec); //command these velocities
        //cout<<"qdot_cmd_vec: "<<q_dot_cmd_vec.transpose()<<endl;
        ros::spinOnce();
        naptime.sleep(); 
    }
}

