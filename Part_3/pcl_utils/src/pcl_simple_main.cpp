//pcl_simple_main.cpp
// uses PclUtils library to compute properties of a selected patch of points
//wsn 3/21/16


#include <pcl_utils/pcl_utils.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_simple_main"); //node name
    ros::NodeHandle nh; 
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    ROS_INFO("going into spin; try selecting points in rviz...");
    ros::spin(); 
    /*

    Eigen::Vector3f plane_normal; 
    double plane_dist;
    while (ros::ok()) {
            if (pclUtils.got_selected_points() ) {
              ROS_INFO("transforming selected points");
              //pclUtils.transform_selected_points_cloud(A_sensor_wrt_torso);
              // reset the selected-points trigger
              pclUtils.reset_got_selected_points();
              //fit a plane to these selected points:
              pclUtils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist); 
              ROS_INFO_STREAM(" normal: "<<plane_normal.transpose()<<"; dist = "<<plane_dist);
            }
              ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
    }
    ROS_INFO("my work is done here!");
    */
}

