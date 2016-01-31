// IM_6dof_svc_client_test.cpp
// use to test IM_6dof.cpp as node interactive_marker_node
// send requests via service calls;
//e.g. 
#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>

#include <example_interactive_marker/ImNodeSvcMsg.h>
const int GET_CURRENT_MARKER_POSE=0;
const int SET_NEW_MARKER_POSE= 1;

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "IM_6dof_svc_client_test");
    ros::NodeHandle nh; //standard ros node handle    
    example_interactive_marker::ImNodeSvcMsg IM_6dof_srv_msg;
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped poseStamped;
    ROS_INFO("setting up a service client of rt_hand_marker");
    ros::ServiceClient IM_6dof_svc_client = nh.serviceClient<example_interactive_marker::ImNodeSvcMsg>("IM6DofSvc");

    IM_6dof_srv_msg.request.cmd_mode = GET_CURRENT_MARKER_POSE;
    
    bool status = IM_6dof_svc_client.call(IM_6dof_srv_msg);    
    ROS_INFO("return status: %d",status);
    ROS_INFO("got current marker pose: x,y,z = %f, %f, %f",
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.x,
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.y,
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z);

    pose = IM_6dof_srv_msg.response.poseStamped_IM_current.pose;
    pose.position.z += 0.1; // increment z value
    poseStamped.pose = pose;
    poseStamped.header.stamp = ros::Time::now();
    IM_6dof_srv_msg.request.cmd_mode = SET_NEW_MARKER_POSE;
    IM_6dof_srv_msg.request.poseStamped_IM_desired = poseStamped;
    ROS_INFO("moving marker up 0.1m");
    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);  
    ROS_INFO("return status: %d",status);
    ROS_INFO("got current marker pose: x,y,z = %f, %f, %f",
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.x,
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.y,
        IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z);    
    return 0;
}
