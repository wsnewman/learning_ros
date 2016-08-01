//uses ROS service to move checkerboard in Gazebo
//turn off gravity, so object does not fall after repositioning
//can tune desired range of displacements and rotations

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <iostream>
#include <string>
using namespace std; 


int main(int argc, char **argv) {
    ros::init(argc, argv, "move_gazebo_model");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state_srv;
    gazebo_msgs::ModelState des_model_state;
    geometry_msgs::Twist twist;
    int ans;
    bool do_skew=false;
     twist.linear.x = 0.0;
     twist.linear.y = 0.0;
     twist.linear.z = 0.0;
     twist.angular.x = 0.0;
     twist.angular.y = 0.0;
     twist.angular.z = 0.0;

    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
    double x_bias = -0.03;
    double y_bias = -0.03;
    double z_bias = 0.2;
    double x,y,z;
    double dx = 0.1;
    double dy = 0.1;
    double dz = 0.15;
     pose.position.x = x;
     pose.position.y = y;
     pose.position.z = z;
     quat.x = 0.0;
     quat.y = 0.0;
     quat.z = 0.0;
     quat.w = 1.0;
     pose.orientation= quat;

    des_model_state.model_name = "small_checkerboard"; 
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";
    double qx,qy,qz,qw;
   //do random displacements and skews
   //cout<<"do skews? (0,1): ";
   //cin>>ans;
   //if (ans) do_skew=true;
    do_skew=true;
   while(ros::ok()) {
	qx = 0.2*(( (rand()%100)/100.0)-0.5); 
	qy = 0.2*(( (rand()%100)/100.0)-0.5);
	qz = 0.2*(( (rand()%100)/100.0)-0.5);
	qw =  0.5; 

	x = x_bias + dx*((rand()%100)/100.0-0.5); 
	y = y_bias + dy*((rand()%100)/100.0-0.5); 
	z = z_bias + dz*((rand()%100)/100.0-0.5); 

    double norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
        quat.x = qx/norm;
        quat.y = qy/norm;
        quat.z = qz/norm;
        quat.w = qw/norm;

	   cout<<"qx, qy, qz, qw= "<<quat.x<<", "<<quat.y<<", "<<quat.z<<", "<<quat.w<<endl;
           cout<<"x,y,z = "<<x<<", "<<y<<", "<<z<<endl;
	   if(do_skew) pose.orientation= quat;
     	   pose.position.x = x;
     	   pose.position.y = y;
     	   pose.position.z = z;
    	   des_model_state.pose = pose;
        set_model_state_srv.request.model_state = des_model_state;
        client.call(set_model_state_srv);
       ros::spinOnce();
       //cout<<"enter 1 to advance, <1 to quit: ";
       //cin>>ans;
       //if (ans<1) return 0;
       ros::Duration(0.5).sleep();
     }
    return 0;
}
