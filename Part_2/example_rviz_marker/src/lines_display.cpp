// lines_display.cpp
// Wyatt Newman, 11/17
// node to assist display of lines in rviz
// this node subscribes to topic "lineseg_vertices", from which it receives 
//geometry_msgs/Point points specifying two vertices of a line segment
// each input ADDS to the current array of line segments (without deletions)
//NOTE: ASSUMES RECEIVED POINTS ARE WITH RESPECT TO "WORLD" FRAME

// To see the result, add a "Marker" display in rviz and subscribe to the marker topic "/lines_display"
// Can test this display node with the test node: "lines_display_test_node"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
//#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

//some globals...
geometry_msgs::Point g_vertex1,g_vertex2;
visualization_msgs::Marker g_line_markers; //lines marker
bool g_trigger_new_lineseg = false; //trigger to add another lineseg

void update_lines(geometry_msgs::Point vertex1, geometry_msgs::Point vertex2) {
    g_line_markers.points.push_back(vertex1);
    g_line_markers.points.push_back(vertex2);
    g_line_markers.header.stamp = ros::Time::now();
    g_trigger_new_lineseg=false;
}


void init_marker_vals(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "/world"; // reference frame for marker coords
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;     
}


void linesegCB(const geometry_msgs::Polygon& vertices) {
    //ROS_DEBUG("got vertices");
    geometry_msgs::Point32 vertex1,vertex2;
    vertex1 = vertices.points[0];
    vertex2 = vertices.points[1];
    //convert to double precision:
    g_vertex1.x = vertex1.x;
    g_vertex1.y = vertex1.y;
    g_vertex1.z = vertex1.z;
    g_vertex2.x = vertex2.x;
    g_vertex2.y = vertex2.y;
    g_vertex2.z = vertex2.z;
    g_trigger_new_lineseg=true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lineseg_displayer"); // this will be the node name;
    ros::NodeHandle nh;

    // subscribe to publications of lineseg vertices
    ros::Subscriber pose_sub = nh.subscribe("lineseg_vertices", 1, linesegCB);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("lines_display", 1);
    g_line_markers.points.clear();
    init_marker_vals(g_line_markers);

    ros::Rate timer(20); //timer to run at 20 Hz



    while (ros::ok()) {
        if(g_trigger_new_lineseg) {
          update_lines(g_vertex1,g_vertex2);
        }
        vis_pub.publish(g_line_markers); //publish the lines
        ros::Duration(0.01).sleep();
        ros::spinOnce(); //let callbacks perform an update
        timer.sleep();
    }
}


