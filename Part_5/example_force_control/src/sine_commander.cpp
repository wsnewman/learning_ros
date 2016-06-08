#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <sensor_msgs/JointState.h>
#include <math.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sine_commander"); // name of this node will be "minimal_publisher2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher cmd_publisher = n.advertise<std_msgs::Float64>("/one_DOF_robot/joint1_velocity_controller/command", 1);
    //std_msgs::JointState js;
    //js.velocity.push_back(0.0);
    //js.position.push_back(0.0);
    
    std_msgs::Float64 v_cmd_float64; //create a variable of type "Float64", 


    //input_float.data = 0.0;
   double v_cmd=0.0;
   double x_cmd=0.0;
   double x_amp=0.0;
   double freq,omega;
   cout<<"enter displacement amplitude: ";
   cin>>x_amp;
   cout<<"enter freq (in Hz): ";
   cin>>freq;
   omega = freq*2.0*M_PI;
   double phase=0;
   double dt = 0.01;
   ros::Rate sample_rate(1/dt); 
  
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        phase+= omega*dt;
        if (phase>2.0*M_PI) phase-=2.0*M_PI;
        x_cmd = x_amp*sin(phase);
        v_cmd = omega*x_amp*cos(phase);
        v_cmd_float64.data = v_cmd;

        cmd_publisher.publish(v_cmd_float64); // publish the value--of type Float64-- 

	sample_rate.sleep(); 
    }
}

