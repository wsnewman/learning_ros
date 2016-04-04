//display_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
//wsn March 2016

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <pcl_ros/point_cloud.h> //to convert between PCL a nd ROS
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher"); //node name
    ros::NodeHandle nh; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    cout<<"enter pcd file name: ";
    string fname;
    cin>>fname;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pcl_clr_ptr) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << pcl_clr_ptr->width * pcl_clr_ptr->height
            << " data points from file "<<fname<<std::endl;

   //publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud;  //here is the ROS-compatible message
    pcl::toROSMsg(*pcl_clr_ptr, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = "camera_depth_optical_frame";
    cout << "view in rviz; choose: topic= pcd; and fixed frame= camera_depth_optical_frame" << endl;
    //publish the ROS-type message on topic "/ellipse"; can view this in rviz
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}
    