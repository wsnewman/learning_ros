#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <sensor_msgs/JointState.h>
#include <math.h>
using namespace std;

//joint command limits:
const double q_max=3.1;
const double q_min=0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sine_commander"); // name of this node 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher pos_cmd_publisher = n.advertise<std_msgs::Float64>("/lidar_wobbler/joint1_position_controller/command", 1);    


   double q_cmd=0.0;
   double q_amp=1.57;
   double freq,omega,q_mid;
   q_mid = (q_max+q_min)/2.0;
   cout<<"enter displacement amplitude: ";
   cin>>q_amp;
   cout<<"enter freq (in Hz): ";
   cin>>freq;
   omega = freq*2.0*M_PI;
   double phase=0;
   double dt = 0.01;
   ros::Rate sample_rate(1/dt); 
   std_msgs::Float64 pos_cmd_float64;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        phase+= omega*dt;
        if (phase>2.0*M_PI) phase-=2.0*M_PI;
        q_cmd = q_amp*sin(phase)+q_mid;
        pos_cmd_float64.data = q_cmd;
        pos_cmd_publisher.publish(pos_cmd_float64); // publish the value--of type Float64-- 
	    sample_rate.sleep(); 
    }
}

