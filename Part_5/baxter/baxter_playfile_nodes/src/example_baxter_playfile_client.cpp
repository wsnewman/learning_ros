// example_baxter_playfile_client
// wsn, September, 2016
// illustrates use of baxter_playfile_service

#include<ros/ros.h>
#include<baxter_playfile_nodes/playfileSrv.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_baxter_playfile_client"); // name this node 
    ros::NodeHandle nh;
    //create a client of playfile_service 
    ros::ServiceClient client = nh.serviceClient<baxter_playfile_nodes::playfileSrv>("playfile_service");
    baxter_playfile_nodes::playfileSrv playfile_srv_msg; //compatible service message
    //set the request to PRE_POSE, per the mnemonic defined in the service message
    playfile_srv_msg.request.playfile_code = baxter_playfile_nodes::playfileSrvRequest::PRE_POSE;

    ROS_INFO("sending pre-pose command to playfile service: ");
    client.call(playfile_srv_msg);
    //blocks here until service call completes...
    ROS_INFO("service responded with code %d", playfile_srv_msg.response.return_code);
    return 0;
}

