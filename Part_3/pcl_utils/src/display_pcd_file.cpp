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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    cout<<"enter pcd file name: ";
    string fname;
    cin>>fname;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr_) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << pclKinect_clr_ptr_->width * pclKinect_clr_ptr_->height
            << " data points from file "<<fname<<std::endl;


   //let's publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud;  //here is the ROS-compatible message
    pcl::toROSMsg(*pclKinect_clr_ptr_, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = "camera_depth_optical_frame";
    
    //publish the ROS-type message on topic "/elipse"; can view this in rviz
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}
    