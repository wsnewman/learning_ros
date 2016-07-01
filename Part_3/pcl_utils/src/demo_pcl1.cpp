//demo_pcl1.cpp
// prompts for a pcd file name, reads the file
//shows how to operate w/ pcl::PointCloud<pcl::PointXYZRGB> objects
//wsn March 2016

#include<ros/ros.h> //generic C++ stuff
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

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrA(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrB(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1_ptrC(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image

    vector<int> indices;
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pcl1_ptrA) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    } 

    cout<<"starting voxel filtering"<<endl;

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pcl1_ptrA); //this is a pointer
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*pcl1_ptrB); //but this is a dereferenced pointer
    cout<<"done voxel filtering"<<endl;
    
    cout<<"num bytes in original cloud data = "<<pcl1_ptrA->points.size()<<endl;
    cout<<"num bytes in filtered cloud data = "<<pcl1_ptrB->points.size()<<endl; // ->data.size()<<endl; 
    
    Eigen::Vector4f xyz_centroid; 
    pcl::compute3DCentroid (*pcl1_ptrB, xyz_centroid);
    
    float curvature;
    Eigen::Vector4f plane_parameters;    
    pcl::computePointNormal(*pcl1_ptrB, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud

    Eigen::Affine3f A(Eigen::Affine3f::Identity());
    pcl::transformPointCloud(*pcl1_ptrB, *pcl1_ptrC, A);    

}
