// object_grabber_as2: 
// wsn, December, 2016
// added behaviors to straddle_object and plan cartesian move
// ObjectGrabber action server w/ ObjectGrabber class

#include<ros/ros.h>
#include <object_grabber/object_grabber2.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_grabber_action_server_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    ObjectGrabber object_grabber_as(&nh); // create an instance of the class "ObjectGrabber", containing an action server

    ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep(); //spinOnce() in a loop w/o timer will crash
    }
    return 0;
}

