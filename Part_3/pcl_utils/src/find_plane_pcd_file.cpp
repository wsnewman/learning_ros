//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
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

#include <pcl_utils/pcl_utils.h>


using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher"); //node name
    ros::NodeHandle nh; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    
    cout<<"enter pcd file name: ";
    string fname;
    cin>>fname;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << pclKinect_clr_ptr->width * pclKinect_clr_ptr->height
            << " data points from file "<<fname<<std::endl;


   //let's publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts",1);
    sensor_msgs::PointCloud2 ros_cloud,ros_planar_cloud;  //here is the ROS-compatible message
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = "camera_depth_optical_frame";
    
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    
    //void PclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,

    cout<<" select a patch of points to find corresponding plane..." <<endl;
    while (!pclUtils.got_selected_points()) {
        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    //cout<<"plane dis: "<<pclUtils.get_patch_dist()<<endl;
    cout<<"got patch; plane params are:"<<pclUtils.get_patch_normal().transpose()<<"; dist = "<<pclUtils.get_patch_dist()<<endl;
    
    
    // Placeholder for the 3x3 covariance matrix at each surface patch
     Eigen::Matrix3f covariance_matrix;
     // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
     Eigen::Vector4f xyz_centroid;
     Eigen::Vector3f centroid;
     
         //void get_selected_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr );
     //void get_selected_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud) {
     pcl::PointCloud<pcl::PointXYZ> selectedPtsCloud;
     pclUtils.get_selected_points(selectedPtsCloud);
     
     //Eigen::Vector3f  PclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
    centroid = pclUtils.compute_centroid(selectedPtsCloud); 
    cout<<"pclUtils centroid: "<<centroid.transpose()<<endl;
            
   // Estimate the XYZ centroid
    pcl::compute3DCentroid (selectedPtsCloud, xyz_centroid);
    cout<<"xyz_centroid: "<<xyz_centroid.transpose()<<endl;

   // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (selectedPtsCloud, xyz_centroid, covariance_matrix);  
    
    
    //create a transform from a frame defined on this plane:
    Eigen::Affine3f A_plane_wrt_camera;
    //    Eigen::Affine3f make_affine_from_plane_params(Eigen::Vector3f plane_normal, double plane_dist);
    A_plane_wrt_camera = pclUtils.make_affine_from_plane_params(pclUtils.get_patch_normal(),pclUtils.get_patch_dist());
    cout<<"A_plane_wrt_camera rotation:"<<endl;
    cout<<A_plane_wrt_camera.linear()<<endl;
    cout<<"origin: "<<A_plane_wrt_camera.translation().transpose()<<endl;
    //void PclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr)
    cout<<"transforming all points to plane coordinates..."<<endl;
    pclUtils.transform_cloud(A_plane_wrt_camera.inverse(),pclKinect_clr_ptr,transformed_cloud_ptr);
    //pcl::io::savePCDFileASCII ("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_)
    pcl::io::savePCDFileASCII ("transformed.pcd", *transformed_cloud_ptr); 
    
    vector<int> indices;
    pclUtils.filter_cloud_z(transformed_cloud_ptr, 0.0, 0.02, indices);
    // void copy_cloud_xyzrgb_indices(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);
    //    void copy_cloud_xyzrgb_indices(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZRGB>::Ptr outputCloud); 
    //pcl::computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);
    float curvature;
    Eigen::Vector4f plane_parameters;
    //    void get_kinect_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud );
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pclUtils.get_kinect_points(cloud);
    pcl::computePointNormal(*pclKinect_clr_ptr, indices,plane_parameters,curvature);
    cout<<"PCL plane_parameters: "<<plane_parameters.transpose()<<endl;

    // populate a new cloud consisting only of the points determined to lie within tolerance on the chosen plane
    pclUtils.copy_cloud_xyzrgb_indices(pclKinect_clr_ptr,indices,plane_pts_ptr);
    
    
    pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert from PCL to ROS type this way
    ros_planar_cloud.header.frame_id = "camera_depth_optical_frame";
    

    cout<<" displaying planar-fit points..." <<endl;
    while (ros::ok()) {
        pubCloud.publish(ros_cloud);
        pubPlane.publish(ros_planar_cloud);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }   
    
    return 0;
}
    