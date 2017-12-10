//linesegs_display_test_node.cpp
//wsn, 11/2017
// use this to illustrate functionality of lines_display, which displays 
//a set of line segments, as defined by pairs of endpoints.
// view result in rviz by adding a marker on topic "lines_display".  
// Set the rviz frame to "world"

//SHOULD transform nodes.  ASSUMES reference frame is "world"

#include<ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include<math.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "lineseg_display_test_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    ros::Publisher polygon_publisher = nh.advertise<geometry_msgs::Polygon>("lineseg_vertices", 1, true);
    geometry_msgs::Polygon lineseg;
    geometry_msgs::Point32 vertex1,vertex2;
    lineseg.points.resize(2);

    vertex1.x=0.0;
    vertex1.y=0.0;
    vertex1.z = 0.0;
    vertex2 = vertex1;
    vertex2.x=1.0;
    
    lineseg.points[0] = vertex1;
    lineseg.points[1] = vertex2;

    while (ros::ok()) {
        vertex1.y = vertex1.y+0.1;
        vertex2.y = vertex2.y+0.1; 
        lineseg.points[0] = vertex1;
        lineseg.points[1] = vertex2;
        polygon_publisher.publish(lineseg);
        ros::Duration(1.0).sleep();
    }
}
