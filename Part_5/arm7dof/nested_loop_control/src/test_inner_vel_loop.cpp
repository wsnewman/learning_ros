#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_inner_vel_loop");
    ros::NodeHandle nh;
    int jnum;
    double omega, amp;
    ros::Publisher jnt_cmd_publisher =
            nh.advertise<std_msgs::Float64MultiArray>("qdes_attractor_vec", 1, true);
    std_msgs::Float64MultiArray qdes_msg;
    for (int i = 0; i < 7; i++) qdes_msg.data.push_back(0.0);
    cout<<" node to command sinusoidal motions to individual joints to test inner_vel_loop controller"<<endl;
    cout<<" you will be prompted for joint number, amplitude and frequency"<<endl;
    cout << "enter jnt num (0-6): ";
    cin >> jnum;
    if ((jnum > 6) || (jnum < 0)) {
        ROS_WARN("wise guy!");
        return 1;
    }

    cout << "enter amplitude, rad: ";
    cin >> amp;

    cout << "enter omega, rad/sec: ";
    cin >> omega;

    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    double dt = 0.01; // 100Hz
    double phase = 0.0;
    while (ros::ok()) {
        phase += omega*dt;
        if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        qdes_msg.data[jnum] = amp * sin(phase);
        jnt_cmd_publisher.publish(qdes_msg);
        ros::Duration(dt).sleep();
    }
}

