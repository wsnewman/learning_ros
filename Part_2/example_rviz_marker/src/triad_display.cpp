// triad_display.cpp
// Wyatt Newman, 8/16
// node to assist display of triads (axes) in rviz
// this node subscribes to topic "triad_display_pose", from which it receives geometry_msgs/PoseStamped poses
// it uses this info to populate and publish axes, using whatever frame_id is in the pose header
// To see the result, add a "Marker" display in rviz and subscribe to the marker topic "/triad_display"
// Can test this display node with the test node: "triad_display_test_node", which generates moving poses
// corresponding to a marker origin spiraling up in z

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

//some globals...
geometry_msgs::Point vertex1;
geometry_msgs::PoseStamped g_stamped_pose;
Eigen::Affine3d g_affine_marker_pose;

// create arrow markers; do this 3 times to create a triad (frame)
visualization_msgs::Marker arrow_marker_x; //this one for the x axis
visualization_msgs::Marker arrow_marker_y; //this one for the y axis
visualization_msgs::Marker arrow_marker_z; //this one for the y axis

//udpdate_arrows() set the frame and 

void update_arrows() {
    geometry_msgs::Point origin, arrow_x_tip, arrow_y_tip, arrow_z_tip;
    Eigen::Matrix3d R;
    Eigen::Quaterniond quat;
    quat.x() = g_stamped_pose.pose.orientation.x;
    quat.y() = g_stamped_pose.pose.orientation.y;
    quat.z() = g_stamped_pose.pose.orientation.z;
    quat.w() = g_stamped_pose.pose.orientation.w;
    R = quat.toRotationMatrix();
    Eigen::Vector3d x_vec, y_vec, z_vec;
    double veclen = 0.2; //make the arrows this long
    x_vec = R.col(0) * veclen;
    y_vec = R.col(1) * veclen;
    z_vec = R.col(2) * veclen;

    //update the arrow markers w/ new pose:
    origin = g_stamped_pose.pose.position;
    arrow_x_tip = origin;
    arrow_x_tip.x += x_vec(0);
    arrow_x_tip.y += x_vec(1);
    arrow_x_tip.z += x_vec(2);
    arrow_marker_x.points.clear();
    arrow_marker_x.points.push_back(origin);
    arrow_marker_x.points.push_back(arrow_x_tip);
    arrow_marker_x.header = g_stamped_pose.header;

    arrow_y_tip = origin;
    arrow_y_tip.x += y_vec(0);
    arrow_y_tip.y += y_vec(1);
    arrow_y_tip.z += y_vec(2);

    arrow_marker_y.points.clear();
    arrow_marker_y.points.push_back(origin);
    arrow_marker_y.points.push_back(arrow_y_tip);
    arrow_marker_y.header = g_stamped_pose.header;

    arrow_z_tip = origin;
    arrow_z_tip.x += z_vec(0);
    arrow_z_tip.y += z_vec(1);
    arrow_z_tip.z += z_vec(2);

    arrow_marker_z.points.clear();
    arrow_marker_z.points.push_back(origin);
    arrow_marker_z.points.push_back(arrow_z_tip);
    arrow_marker_z.header = g_stamped_pose.header;
}

//init persistent params of markers, then variable coords    

void init_markers() {
    //initialize stamped pose for at a legal (if boring) pose
    g_stamped_pose.header.stamp = ros::Time::now();
    g_stamped_pose.header.frame_id = "world";
    g_stamped_pose.pose.position.x = 0;
    g_stamped_pose.pose.position.y = 0;
    g_stamped_pose.pose.position.z = 0;
    g_stamped_pose.pose.orientation.x = 0;
    g_stamped_pose.pose.orientation.y = 0;
    g_stamped_pose.pose.orientation.z = 0;
    g_stamped_pose.pose.orientation.w = 1;

    //the following parameters only need to get set once
    arrow_marker_x.type = visualization_msgs::Marker::ARROW;
    arrow_marker_x.action = visualization_msgs::Marker::ADD; //create or modify marker
    arrow_marker_x.ns = "triad_namespace";
    arrow_marker_x.lifetime = ros::Duration(); //never delete
    // make the arrow thin
    arrow_marker_x.scale.x = 0.01;
    arrow_marker_x.scale.y = 0.01;
    arrow_marker_x.scale.z = 0.01;
    arrow_marker_x.color.r = 1.0; // red, for the x axis
    arrow_marker_x.color.g = 0.0;
    arrow_marker_x.color.b = 0.0;
    arrow_marker_x.color.a = 1.0;
    arrow_marker_x.id = 0;
    arrow_marker_x.header = g_stamped_pose.header;

    //y and z arrow params are the same, except for colors
    arrow_marker_y = arrow_marker_x;
    arrow_marker_y.color.r = 0.0;
    arrow_marker_y.color.g = 1.0; //green for y axis
    arrow_marker_y.color.b = 0.0;
    arrow_marker_y.color.a = 1.0;
    arrow_marker_y.id = 1;

    arrow_marker_z = arrow_marker_x;
    arrow_marker_z.id = 2;
    arrow_marker_z.color.r = 0.0;
    arrow_marker_z.color.g = 0.0;
    arrow_marker_z.color.b = 1.0; //blue for z axis
    arrow_marker_z.color.a = 1.0;
    //set the poses of the arrows based on g_stamped_pose
    update_arrows();
}

void poseCB(const geometry_msgs::PoseStamped &pose_msg) {
    ROS_DEBUG("got pose message");
    //ROS_INFO("got pose message");
    g_stamped_pose.header = pose_msg.header;
    g_stamped_pose.pose = pose_msg.pose;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "triad_display"); // this will be the node name;
    ros::NodeHandle nh;

    // subscribe to stamped-pose publications
    ros::Subscriber pose_sub = nh.subscribe("triad_display_pose", 1, poseCB);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("triad_display", 1);
    init_markers();

    ros::Rate timer(20); //timer to run at 20 Hz



    while (ros::ok()) {
        update_arrows();
        vis_pub.publish(arrow_marker_x); //publish the marker
        ros::Duration(0.01).sleep();
        vis_pub.publish(arrow_marker_y); //publish the marker
        ros::Duration(0.01).sleep();
        vis_pub.publish(arrow_marker_z); //publish the marker
        ros::spinOnce(); //let callbacks perform an update
        timer.sleep();
    }
}


