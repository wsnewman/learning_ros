//make_and_display_ellipse.cpp
// example of creating a point cloud and publishing it for rviz display
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
    ros::init(argc, argv, "ellipse"); //node name
    ros::NodeHandle nh; 

   // ------------------------------------
   // -----Create example point cloud-----
   // ------------------------------------
   // create some point-cloud objects to hold data
   pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>); //no color
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); //colored
   std::cout << "Generating example point-cloud ellipse.\n\n";
   //example ellipse, borrowed from: http://docs.ros.org/hydro/api/pcl/html/pcl__visualizer__demo_8cpp_source.html
   // make an ellipse extruded along the z-axis. The color for
   // the XYZRGB cloud will gradually go from red to green to blue.
   uint8_t r(255), g(15), b(15);
   for (float z(-1.0); z <= 1.0; z += 0.05)
   {
     for (float angle(0.0); angle <= 360.0; angle += 5.0)
     {
       pcl::PointXYZ basic_point; // simple points have x,y,z, but no color
       basic_point.x = 0.5 * cosf (pcl::deg2rad(angle)); //cosf is cosine, operates on and returns single-precision floats
       basic_point.y = sinf (pcl::deg2rad(angle));
       basic_point.z = z;
       basic_cloud_ptr->points.push_back(basic_point); //append this point to the vector of points
 
       pcl::PointXYZRGB point; //colored point clouds also have RGB values
       point.x = basic_point.x;
       point.y = basic_point.y;
       point.z = basic_point.z;
       //color is encoded strangely, but efficiently.  Stored as a 4-byte "float", but
       // interpreted as individual byte values for 3 colors
       // bits 0-7 are blue value, bits 8-15 are green, bits 16-23 are red; 
       uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
               static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
       point.rgb = *reinterpret_cast<float*>(&rgb);
       point_cloud_ptr->points.push_back (point);
     }
     if (z < 0.0)
     {
       r -= 12;
       g += 12;
     }
     else
     {
       g -= 12;
       b += 12;
     }
   }
   //height=1 implies this is not an "ordered" point cloud
   basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
   basic_cloud_ptr->height = 1;
   basic_cloud_ptr->header.frame_id = "camera"; // need to assign a frame id
   
   point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
   point_cloud_ptr->height = 1;  
   point_cloud_ptr->header.frame_id = "camera";  

   //let's publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ellipse", 1);
    sensor_msgs::PointCloud2 ros_cloud;  //here is the ROS-compatible message
    pcl::toROSMsg(*point_cloud_ptr, ros_cloud); //convert from PCL to ROS type this way
    
    //publish the ROS-type message on topic "/elipse"; can view this in rviz
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}
    