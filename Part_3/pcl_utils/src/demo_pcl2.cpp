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
#include <pcl/PCLPointCloud2.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/filter.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_utils/pcl_utils.h>
#include <pcl-1.7/pcl/pcl_base.h>  //a local library with some utility fncs


int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PCLPointCloud2::Ptr pcl2_ptrA (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl2_ptrB (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl2_ptrC (new pcl::PCLPointCloud2 ());
    

    vector<int> indices;
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    pcl::PCDReader reader;
    reader.read (fname, *pcl2_ptrA);
    //cloud->header.frame_id = "camera_depth_optical_frame";
 
    cout<<"starting voxel filtering"<<endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(pcl2_ptrA); //this is a pointer
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*pcl2_ptrB); //but this is a dereferenced pointer
    cout<<"done voxel filtering"<<endl;
    
    cout<<"num bytes in original cloud data = "<<pcl2_ptrA->data.size()<<endl;
    cout<<"num bytes in filtered cloud data = "<<pcl2_ptrB->data.size()<<endl; // ->data.size()<<endl; 

    pcl::PassThrough<pcl::PointXYZ> pass;
    //pcl::PassThrough<pcl::PCLPointCLoud2> pass;
    // Create a filtering object
    //pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;    
    //pcl::NormalEstimation<pcl::PCLPointCloud2,pcl::PCLPointCloud2> Ne;
    //pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
    //the next 3 functions do NOT compile:
    Eigen::Vector4f xyz_centroid; 
    //pcl::Filter<pcl::PCLPointCloud2> filter(pcl::PCLPointCloud2); // filter;
    //filter.setInputCloud(*pcl2_ptrB);
    //filter (PCLPointCloud2 &output)
    //Calls the filtering method and returns the filtered dataset in output
    //pcl::compute3DCentroid (*pcl2_ptrB, xyz_centroid);
    
    float curvature;
    Eigen::Vector4f plane_parameters;    
    //pcl::computePointNormal(*pcl2_ptrB, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud

    Eigen::Affine3f A(Eigen::Affine3f::Identity());
    //pcl::transformPointCloud(*pcl2_ptrB, *pcl2_ptrC, A);    

}
