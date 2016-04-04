//pcd_snapshot.cpp
// example of saving a kinect snapshot to a pcd file
// need to connect "Kinect" and start it with: roslaunch freenect_launch freenect.launch

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/ros/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

using namespace std;

bool got_kinect_image = false; //snapshot indicator
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
        ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
        got_kinect_image = true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_snapshot_main"); //node name
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_subscriber = nh.subscribe("/camera/depth_registered/points", 1, kinectCB);

    //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got snapshot; saving to file kinect_snapshot.pcd");
    pcl::io::savePCDFile("kinect_snapshot.pcd", *pclKinect_clr_ptr, true);

    return 0;
}
