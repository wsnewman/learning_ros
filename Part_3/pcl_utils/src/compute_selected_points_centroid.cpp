//compute_selected_points_centroid.cpp
//wsn 8/2/16
// subscribed to selected_points topic; compute centroid of received points and print to screen

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>


using namespace std;
// this callback wakes up when a new "selected Points" message arrives
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelectedPoints_ptr;
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    ROS_INFO("RECEIVED NEW PATCH");

    pcl::fromROSMsg(*cloud, *g_pclSelectedPoints_ptr);
    ROS_INFO("patch dimensions: %d * %d points", g_pclSelectedPoints_ptr->width, g_pclSelectedPoints_ptr->height);
    

    //ROS_INFO("frame_id = %s",pclSelectedPoints_ptr_->header.frame_id);
    //std::cout<<"frame_id ="<<g_pclSelectedPoints_ptr->header.frame_id<<endl;
    ROS_INFO_STREAM("frame_id = "<<g_pclSelectedPoints_ptr->header.frame_id<<endl);
    
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = g_pclSelectedPoints_ptr->points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = g_pclSelectedPoints_ptr->points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }   

    Eigen::Vector3f centroid = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    // compute the centroid of the selected points
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid += points_mat.col(ipt); //add all the column vectors together
    }
    centroid /= npts; //divide by the number of points to get the centroid   
    ROS_INFO("centroid of selected points is: (%f, %f, %f)",centroid(0),centroid(1),centroid(2));
         /**/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "compute_selected_points_centroid"); //node name
    ros::NodeHandle nh;
    
    // subscribe to "selected_points", which is published by Rviz tool
    ros::Subscriber selected_points_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    g_pclSelectedPoints_ptr = pclSelectedPoints_ptr;
    ROS_INFO(" select a patch of points to find the selected-points centroid...");
    //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    while (ros::ok()) {
        ros::spinOnce(); //callback does all the work
        ros::Duration(0.1).sleep();
    }

    return 0;
}
