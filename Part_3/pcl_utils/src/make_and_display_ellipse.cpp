//make_and_display_ellipse.cpp
//example of creating a point cloud and publishing it for rviz display
//wsn March 2016

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/ros/conversions.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>


using namespace std;

//a function to populate a pointCloud and a colored pointCloud;
// provide pointers to these, and this function will fill them with data
void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr) {
    //example ellipse, borrowed from: http://docs.ros.org/hydro/api/pcl/html/pcl__visualizer__demo_8cpp_source.html
    // make an ellipse extruded along the z-axis. The color for
    // the XYZRGB cloud will gradually go from red to green to blue.
    
    uint8_t r(255), g(15), b(15); //declare and initialize red, green, blue component values
    
    //here are "point" objects that are compatible as building-blocks of point clouds
    pcl::PointXYZ basic_point; // simple points have x,y,z, but no color
    pcl::PointXYZRGB point; //colored point clouds also have RGB values

    for (float z = -1.0; z <= 1.0; z += 0.05) //build cloud in z direction
    {
        // color is encoded strangely, but efficiently.  Stored as a 4-byte "float", but
        // interpreted as individual byte values for 3 colors
        // bits 0-7 are blue value, bits 8-15 are green, bits 16-23 are red; 
        // Can build the rgb encoding with bit-level operations:
        uint32_t rgb = (static_cast<uint32_t> (r) << 16 |
                static_cast<uint32_t> (g) << 8 | static_cast<uint32_t> (b));
        
        // and encode these bits as a single-precision (4-byte) float:
        float rgb_float = *reinterpret_cast<float*> (&rgb);
        
        //using fixed color and fixed z, compute coords of an ellipse in x-y plane
        for (float ang = 0.0; ang <= 2.0 * M_PI; ang += 2.0 * M_PI / 72.0) {
            //choose minor axis length= 0.5, major axis length = 1.0
            // compute and fill in components of point
            basic_point.x = 0.5 * cosf(ang); //cosf is cosine, operates on and returns single-precision floats
            basic_point.y = sinf(ang);
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point); //append this point to the vector of points

            //use the same point coordinates for our colored pointcloud      
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            //but also add rgb information
            point.rgb = rgb_float; //*reinterpret_cast<float*> (&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0) //alter the color smoothly in the z direction
        {
            r -= 12; //less red
            g += 12; //more green
        } else {
            g -= 12; // for positive z, lower the green
            b += 12; // and increase the blue
        }
    }
    
    //these will be unordered point clouds, i.e. a random bucket of points
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1; //height=1 implies this is not an "ordered" point cloud
    basic_cloud_ptr->header.frame_id = "camera"; // need to assign a frame id

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->header.frame_id = "camera";        

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipse"); //node name
    ros::NodeHandle nh;

    // create some point-cloud objects to hold data
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //no color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //colored

    cout << "Generating example point-cloud ellipse.\n\n";
    cout << "view in rviz; choose: topic= ellipse; and fixed frame= camera" << endl;
    
    // -----use fnc to create example point clouds: basic and colored-----
    make_clouds(basic_cloud_ptr, point_cloud_clr_ptr);

    // we now have "interesting" point clouds in basic_cloud_ptr and point_cloud_clr_ptr
    //let's publish the colored point cloud in a ROS-compatible message; here's a publisher...
    // we'll publish to topic "ellipse"
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ellipse", 1);
    sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible pointCloud message
    //we'll publish the colored point cloud; 
    pcl::toROSMsg(*point_cloud_clr_ptr, ros_cloud); //convert from PCL to ROS type this way

    //publish the ROS-type message; can view this in rviz on topic "/ellipse"
    //BUT need to set the Rviz fixed frame to "camera"
    while (ros::ok()) {
        pubCloud.publish(ros_cloud);
        ros::Duration(0.5).sleep(); //keep refreshing the publication periodically
    }
    return 0;
}
