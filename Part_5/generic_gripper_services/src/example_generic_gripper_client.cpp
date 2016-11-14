//example_generic_gripper_client.cpp
//wsn Nov 2016
//example of how to communicate with generic gripper interface,
//requires that a gripper service is running
#include<ros/ros.h>

#include<generic_gripper_services/genericGripperInterface.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "generic_gripper_interface_client");
    ros::NodeHandle n;
    ros::ServiceClient client = 
      n.serviceClient<generic_gripper_services::genericGripperInterface>("generic_gripper_svc");
    generic_gripper_services::genericGripperInterface srv;
    srv.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::TEST_PING;
    ROS_INFO("sending a test ping");
    bool success=false;
    while (!success) {
         client.call(srv); 
         success = srv.response.success;
         ROS_INFO("retrying...is the gripper service running?");
         ros::Duration(0.5).sleep();
         ros::spinOnce();
    }
    ROS_INFO("got test ping response");
    
    ROS_INFO("try sending release command: ");
    srv.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
    client.call(srv); 
    success = srv.response.success;
    if (success) { ROS_INFO("responded w/ success"); }
    else {ROS_WARN("responded with failure"); }

    ROS_INFO("try sending GRASP command: ");
    srv.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::GRASP;
    client.call(srv); 
    success = srv.response.success;
    if (success) { ROS_INFO("responded w/ success"); }
    else {ROS_WARN("responded with failure"); }

    return 0;
}    
