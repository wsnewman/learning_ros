# pcl_utils
This package contains some examples of using PCL.  
make_and_display_ellipse shows how to manually populate a point-cloud object, convert the PCL object to a ROS pointCloud message, and publish this message, viewable by Rviz.

pcd_snapshot is an example utility that exploits the pcl_utils library.  It waits for valid Kinect pointcloud message, then saves this message to disk as a 
PCD file by the name of kinect_snapshot.pcd.

display_pcd_file prompts for a file name, loads the file, converts it to a ROS message, and publishes this message for display in Rviz.  The
topic is "pcd" and the frame is "camera_depth_optical_frame".

find_plane_pcd_file illustrates use of pclUtils functions, as well as use of native PCL functions.  It prompts for a PCD file name, reads the file,
and (like display_pcd) converts to a ROS message and publishes it (topic is "pcd" and the frame is "camera_depth_optical_frame").  It then prompts the
user to use the "Publish Selected Points" tool to select a patch, presumed to be on a planar surface of interest.  Planar parameters of the
selected patch are returned and displayed.  The pcl::compute3DCentroid() is compared to pclUtils.compute_centroid().  

A coordinate frame is defined based on the selected patch, where the z-axis is the surface normal.  All of the original points are transformed to this frame,
then within this transformed cloud, points with near-zero z-values are identified.  These points are identified in terms of a list of indices.
These indices are used to index into the original cloud using  pclUtils.copy_cloud_xyzrgb_indices(), then these points are published on the topic "planar_pts",
viewable within Rviz.

## Example usage
An example pcd file is contained in "kinect_clr_snapshot" within the repository, Part_3/jinx_pcd.  This file is ASCII and human readable (e.g. using gedit).
Start up a roscore, start up Rviz.  
In another terminal, navigate to the directory containing the point-cloud file.  Run display_pcd_file or find_plane_pcd_file.  Respond to the prompt
with a valid PCD file name (e.g. kinect_clr_snapshot).


    