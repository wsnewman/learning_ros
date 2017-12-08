//similar to example_rviz_marker, but using lines instead of spheres

#include <ros/ros.h> 
#include <visualization_msgs/Marker.h> // need this for publishing markers
#include <geometry_msgs/Point.h> //data type used for markers
#include <string.h>
#include <stdio.h>  
#include <example_rviz_marker/SimpleFloatSrvMsg.h> //a custom message type defined in this package
using namespace std;

//set these two values by service callback, make available to "main"
double g_z_height = 0.0;
bool g_trigger = true;

//a service to prompt a new display computation.
// E.g., to construct a plane at height z=1.0, trigger with: 
// rosservice call rviz_marker_svc 1.0

bool displaySvcCB(example_rviz_marker::SimpleFloatSrvMsgRequest& request,
	example_rviz_marker::SimpleFloatSrvMsgResponse& response) {
    g_z_height = request.request_float32;
    ROS_INFO("example_rviz_marker: received request for height %f", g_z_height);
    g_trigger = true; // inform "main" a new computation is desired
    response.resp=true;
    return true;
}

void init_marker_vals(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "/world"; // reference frame for marker coords
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    // use SPHERE_LIST for a group of markers
    marker.type = visualization_msgs::Marker::LINE_LIST; // SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y,   				    
    //        marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below

    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02; //ignored
    marker.scale.z = 0.02; //ignored
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;     
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_rviz_marker");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("example_marker_topic", 0);
    visualization_msgs::Marker marker; // instantiate a marker object
    geometry_msgs::Point point; // points will be used to specify where the markers go
    
    //set up a service to compute marker locations on request
    ros::ServiceServer service = nh.advertiseService("rviz_marker_svc", displaySvcCB);

    init_marker_vals(marker);
    
    double z_des;

    // build a wall of markers; set range and resolution
    double x_min = -1.0;
    double x_max = 1.0;
    double y_min = -1.0;
    double y_max = 1.0;
    double dx_des = 0.1;
    double dy_des = 0.1;

    double line_length = 1.0;

    while (ros::ok()) {
        if (g_trigger) {  // did service get request for a new computation?
            g_trigger = false; //reset the trigger from service
            z_des = g_z_height; //use z-value from service callback
            ROS_INFO("constructing plane of markers at height %f",z_des);
    	    marker.header.stamp = ros::Time();
            marker.points.clear(); // clear out this vector
            double y_des = y_min;
            for (double x_des = x_min; x_des < x_max; x_des += dx_des) {
               // for (double y_des = y_min; y_des < y_max; y_des += dy_des) {
                        point.x = x_des;
                        point.y = y_des;
                        point.z = z_des;
                        marker.points.push_back(point);
                        point.y = y_des+line_length; //second vertex of line segment
			marker.points.push_back(point);
               // }
            }
        }
     ros::Duration(0.1).sleep();
    //ROS_INFO("publishing...");
    vis_pub.publish(marker);
    ros::spinOnce();       
    }
    return 0;
}



