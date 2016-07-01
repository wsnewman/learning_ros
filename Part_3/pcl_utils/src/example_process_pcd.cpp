//example_process_pcd.cpp
// example of opening a kinect snapshot pcd file and interpreting it
//wsn March 2016


#include <pcl_utils/pcl_utils.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "process_pcd"); //node name
    ros::NodeHandle nh; 
    string fname;
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    /*
    //load a named pcd file
    cout<<"enter file name (in current directory): ";
    cin>>fname;
    //if (0!= pclUtils.read_clr_pcd_file(fname)) //this version for color
    if (0!= pclUtils.read_pcd_file(fname)){  
        ROS_ERROR("error loading file");
    }
    */
   // ------------------------------------
   // -----Create example point cloud-----
   // ------------------------------------
   pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
   std::cout << "Genarating example point clouds.\n\n";
   // We're going to make an ellipse extruded along the z-axis. The colour for
   // the XYZRGB cloud will gradually go from red to green to blue.
   uint8_t r(255), g(15), b(15);
   for (float z(-1.0); z <= 1.0; z += 0.05)
   {
     for (float angle(0.0); angle <= 360.0; angle += 5.0)
     {
       pcl::PointXYZ basic_point;
       basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
       basic_point.y = sinf (pcl::deg2rad(angle));
       basic_point.z = z;
       basic_cloud_ptr->points.push_back(basic_point);
 
       pcl::PointXYZRGB point;
       point.x = basic_point.x;
       point.y = basic_point.y;
       point.z = basic_point.z;
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
   basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
   basic_cloud_ptr->height = 1;
   point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
   point_cloud_ptr->height = 1;    

   /*
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //color
   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //no color
   //pcl::io::loadPCDFile<PointXYZ>(fname.c_str(),*cloud);
   
   //pclUtils.get_kinect_points(cloud);
   //cout<<"main: got cloud with "<<cloud->points.size()<<" points"<<endl;
   cout<<"starting viewer..."<<endl;
   
   //viewer.showCloud (cloud);
   //viewer.showCloud (basic_cloud_ptr); //point_cloud_ptr
   viewer.showCloud (point_cloud_ptr);
   while (!viewer.wasStopped ())
   {
   }   
    * */
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/elipse", 1);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*point_cloud_ptr, ros_cloud); 
    ros_cloud.header.frame_id="camera";
    
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}
    