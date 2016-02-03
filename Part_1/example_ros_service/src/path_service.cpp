//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// this could be useful for 

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(example_ros_service::PathSrvRequest& request, example_ros_service::PathSrvResponse& response)
{
    
    ROS_INFO("callback activated");
    int npts = request.nav_path.poses.size();
    ROS_INFO("received path request with %d poses",npts);
 
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("path_service", callback);
  ROS_INFO("Ready accept paths.");
  ros::spin();

  return 0;
}
