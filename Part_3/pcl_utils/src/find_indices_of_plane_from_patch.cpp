//find_indices_of_plane_from_patch.cpp

// given a patch of point-cloud points, patch_cloud, that are presumably nearly coplanar
// this function finds all other points in the cloud input_cloud that
// are nearly co-planar with the input hint.  The result is returned in a vector
// of point indices, specifying which points in input_cloud qualify as co-planar
// Illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// and pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()

//wsn March 2016

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

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


using namespace std;
PclUtils *g_pcl_utils_ptr;

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices) {

    float curvature;
    Eigen::Vector4f plane_parameters;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;

    //next, define a coordinate frame on the plane fitted to the patch.
    // choose the z-axis of this frame to be the plane normal--but enforce that the normal
    // must point towards the camera
    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
    cout << "A_plane_wrt_camera rotation:" << endl;
    cout << A_plane_wrt_camera.linear() << endl;
    cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;

    //next, transform all points in input_cloud into the plane frame.
    //the result of this is, points that are members of the plane of interest should have z-coordinates
    // nearly 0, and thus these points will be easy to find
    cout << "transforming all points to plane coordinates..." << endl;
    //Transform each point in the given point cloud according to the given transformation. 
    //pcl fnc: pass in ptrs to input cloud, holder for transformed cloud, and desired transform
    //note that if A contains a description of the frame on the plane, we want to xform with inverse(A)
    pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());

    //now we'll use some functions from the pcl filter library; 
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(-0.02, 0.02); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}
