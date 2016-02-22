#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//node to receive desired states and republish the twist as cmd_vel commands
ros::Publisher g_twist_republisher;

//simply copy the desired twist and republish it to cmd_vel
void desStateCallback(const nav_msgs::Odometry& des_state) {
    geometry_msgs::Twist twist = des_state.twist.twist;
    g_twist_republisher.publish(twist);    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "open_loop_controller"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    g_twist_republisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber des_state_subscriber = n.subscribe("/desState",1,desStateCallback); 
    ros::spin();
}

