//sample program to transform lidar data--for illustration only
//better: use laser_pipeline, see http://wiki.ros.org/laser_pipeline
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h> //ALWAYS need to include this

#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <sensor_msgs/LaserScan.h>
using namespace std;

//these are globals
tf::TransformListener *g_listener_ptr; //a transform listener
XformUtils xformUtils; //instantiate an object of XformUtils
vector <Eigen::Vector3d> g_pt_vecs_wrt_lidar_frame; //will hold 3-D points in LIDAR frame
vector <Eigen::Vector3d> g_pt_vecs_wrt_world_frame; //will hold 3_D points in world frame

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    //if here, then a new LIDAR scan has been received
    // get the transform from LIDAR frame to world frame
    tf::StampedTransform stfLidar2World;
    //specialized for lidar_wobbler; more generally, use scan_in->header.frame_id
    g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);
    //extract transform from transformStamped:
    tf::Transform tf = xformUtils.get_tf_from_stamped_tf(stfLidar2World);    
    //stfLidar2World is only the pose of the LIDAR at the LAST ping...
    //better would be to consider separate transforms for each ping
    //using the above transform for all points is adequate approx if LIDAR is wobbling slowly enough
    Eigen::Affine3d affine_tf,affine_tf_inv; //can use an Eigen type "affine" object for transformations
    //convert transform to Eigen::Affine3d
    affine_tf = xformUtils.transformTFToAffine3d(tf); //can use this to transform points to world frame
    affine_tf_inv = affine_tf.inverse();
    vector <float> ranges = scan_in->ranges; //extract all the radius values from scan
    int npts = ranges.size(); //see how many pings there are in the scan; expect 181 for wobbler model
    g_pt_vecs_wrt_lidar_frame.clear();
    g_pt_vecs_wrt_world_frame.clear();

    ROS_INFO("received %d ranges: ", npts);
    double start_ang = scan_in->angle_min; //get start and end angles from scan message
    double end_ang = scan_in->angle_max;   //should be -90 deg to +90 deg
    double d_ang = (end_ang - start_ang) / (npts - 1); //samples are at this angular increment
    ROS_INFO("d_ang = %f", d_ang);
    Eigen::Vector3d vec; //var to hold one point at a time
    vec[2] = 0.0; //all pings in the LIDAR frame are in x-y plane, so z-component is 0
    
    double ang;
    for (int i = 0; i < npts; i++) {
        if (ranges[i] < 5.0) { //only transform points within 5m
            //if range is too long, LIDAR is nearly parallel to the ground plane, so skip this ping
            ang = start_ang + i*d_ang; //polar angle of this ping
            vec[0] = ranges[i] * cos(ang); //convert polar coords to Cartesian coords
            vec[1] = ranges[i] * sin(ang);
            g_pt_vecs_wrt_lidar_frame.push_back(vec); //save the valid 3d points
        }
    }
    int npts3d = g_pt_vecs_wrt_lidar_frame.size(); //this many points got converted
    ROS_INFO("computed %d 3-D pts w/rt LIDAR frame", npts3d);
    g_pt_vecs_wrt_world_frame.resize(npts3d); 

    //transform the points to world frame:
    //do this one point at a time; alternatively, could have listed all points
    //as column vectors in a single matrix, then do a single multiply to convert the
    //entire matrix of points to the world frame
    for (int i = 0; i < npts3d; i++) {
        g_pt_vecs_wrt_world_frame[i] = affine_tf * g_pt_vecs_wrt_lidar_frame[i];
    }

    //the points in g_pt_vecs_wrt_world_frame are now in Cartesian coordinates
    // points in this frame are easier to interpret
    
    //can now analyze these points to interpret shape of objects on the ground plane
    //but for this example, simply display the z values w/rt world frame:    
    for (int i = 0; i < npts3d; i++) {
        vec = g_pt_vecs_wrt_world_frame[i]; //consider the i'th point
        if (vec[2]< 0.1) {
          ROS_INFO("(x,y,z) = (%6.3f, %6.3f, %6.3f)", vec[0],vec[1],vec[2]);
        }
        else {
            ROS_WARN("(x,y,z) = (%6.3f, %6.3f, %6.3f)", vec[0],vec[1],vec[2]);
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_wobbler_transformer"); //node name
    ros::NodeHandle nh;

    g_listener_ptr = new tf::TransformListener;
    tf::StampedTransform stfLidar2World;
    bool tferr = true;
    ROS_INFO("trying to get tf of lidar_link w/rt world: ");
    //topic /scan has lidar data in frame_id: lidar_link
    while (tferr) {
        tferr = false;
        try {
            g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("transform received; ready to process lidar scans");
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, scanCallback);
    ros::spin(); //let the callback do all the work

    return 0;
}
