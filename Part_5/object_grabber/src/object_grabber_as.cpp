// object_grabber_as: 
// wsn, August, 2016
// illustrates use of ObjectGrabber action server

#include<ros/ros.h>
#include <object_grabber/object_grabber.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_grabber_action_server_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    ObjectGrabber object_grabber_as(&nh); // create an instance of the class "ObjectGrabber", containing an action server

    ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }
    return 0;
}

