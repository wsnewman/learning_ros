//pcd_snapshot.cpp
// example of saving a kinect snapshot to a pcd file
// need to connect "Kinect" and start it with: roslaunch freenect_launch freenect.launch
//wsn 3/21/16


#include <pcl_utils/pcl_utils.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_snapshot_main"); //node name
    ros::NodeHandle nh; 
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!pclUtils.got_kinect_cloud()) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep(); //  ros::Duration(0.5).sleep();
    }
    ROS_INFO("got snapshot; saving to file kinect_snapshot.pcd");
    pclUtils.save_kinect_clr_snapshot();//save_kinect_clr_snapshot
    
    return 0;
}
    