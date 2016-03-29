//ransac_main.cpp
// example using ransac algorithm to find a plane
//wsn 3/21/16


#include <pcl_utils/pcl_utils.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PointIndices.h>
#include <pcl-1.7/pcl/PCLHeader.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_simple_main"); //node name
    ros::NodeHandle nh; 
    pcl::PointCloud<pcl::PointXYZ> pt_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ransacCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!pclUtils.got_kinect_cloud()) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep(); //  ros::Duration(0.5).sleep();
    }
    ROS_INFO("got snapshot");    
    //get_kinect_points
    
    pclUtils.get_kinect_points(cloudPtr);  
    //xxxxx
    int npts = cloudPtr->points.size();
    ROS_INFO("got cloud with %d points",npts);
    cout<<"frame id: "<<cloudPtr->header.frame_id<<endl;
   //std::vector<int> inliers;

  // here's the RANSAC code:
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setInputCloud (cloudPtr);
  seg.segment (*inliers, *coefficients);
  //done w/ RANSAC code
  
  //output and display results:
  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  
   ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ransac_pts", 1, true);
   
   //PointCloud<pcl::PointXYZ>::Ptr outputCloud;
       int n_inliers = inliers->indices.size(); //how many points to extract?
       ROS_INFO("found %d inliers",n_inliers);
    ransacCloud->header = cloudPtr->header;
    ransacCloud->is_dense = cloudPtr->is_dense;
    ransacCloud->width = n_inliers;
    ransacCloud->height = 1;

    cout << "copying cloud w/ npts =" << n_inliers << endl;
    ransacCloud->points.resize(n_inliers);
    for (int i = 0; i < n_inliers; ++i) {
        ransacCloud->points[i].getVector3fMap() = cloudPtr->points[inliers->indices[i]].getVector3fMap();
    }
   

    sensor_msgs::PointCloud2 ros_cloud;
    
    while (ros::ok()) {
        pcl::toROSMsg(*ransacCloud, ros_cloud);
        pointcloud_publisher.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.5).sleep(); //  ros::Duration(0.5).sleep();
    }   

}

