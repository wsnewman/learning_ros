//lidar_wobbler.cpp
//wsn 8/4/2016
#include <laser_geometry.h>
#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>


laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/world",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/world",*scan_in,cloud,listener_);

  // Do something with cloud.
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_wobbler_transformer"); //node name
    ros::NodeHandle nh;
    ros::Subscriber lidar_subscriber = nh.subscribe("/laser/scan",1,scanCallback);
    while (ros::ok()) {
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.01).sleep();
    }

    return 0;
}

