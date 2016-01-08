#include <ros/ros.h>
#include <example_ros_msg/ExampleMessage.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_ros_message_publisher"); // name of this node 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<example_ros_msg::ExampleMessage>("example_topic", 1);
    //"example_topic" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    example_ros_msg::ExampleMessage  my_new_message;
    //create a variable of type "example_msg", 
    // as defined in this package
   
   ros::Rate naptime(1.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

   // put some data in the header.  Do: rosmsg show std_msgs/Header
   //  to see the definition of "Header" in std_msgs
    my_new_message.header.stamp = ros::Time::now(); //set the time stamp in the header;
    my_new_message.header.seq=0; // call this sequence number zero
    my_new_message.header.frame_id = "base_frame"; // would want to put true reference frame name here, if needed for coord transforms
    my_new_message.demo_int= 1;
    my_new_message.demo_double=100.0;
    
    double sqrt_arg;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
       my_new_message.header.seq++; //increment the sequence counter
       my_new_message.header.stamp = ros::Time::now(); //update the time stamp
       my_new_message.demo_int*=2.0; //double the integer in this field
       sqrt_arg = my_new_message.demo_double;
       my_new_message.demo_double = sqrt(sqrt_arg);
       
        my_publisher_object.publish(my_new_message); // publish the data in new message format on topic "example_topic"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep(); 
    }
}

