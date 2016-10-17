#include <ros/ros.h>
#include <baxter_core_msgs/HeadPanCommand.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_head_pan"); 
    ros::NodeHandle n; 
    //create a publisher to send commands to Baxter's head pan
    ros::Publisher head_pan_pub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1);

    baxter_core_msgs::HeadPanCommand headPanCommand; //message type for head-pan control
    headPanCommand.target = 0.0;
    headPanCommand.enable_pan_request=1;
    //prompt user for amplitude and frequency of oscillating head pan
    double amp,freq;
    cout<<"enter pan amplitude (rad): ";
    cin>>amp;
    cout<<"enter pan freq (Hz): ";
    cin>>freq;
    double dt = 0.01;
    ros::Rate timer(1/dt);
    double phase=0.0;
    double theta;
    
    // oscillate head pan indefinitely
    while (ros::ok()) 
    {
        phase+= M_PI*2.0*freq*dt;
        if (phase>2.0*M_PI) phase-= 2.0*M_PI;
        theta = amp*sin(phase);
        headPanCommand.target = theta;
        head_pan_pub.publish(headPanCommand);
        timer.sleep();
    }
}

