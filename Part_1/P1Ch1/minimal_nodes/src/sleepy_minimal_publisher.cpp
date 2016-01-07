#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_publisher2"); // name of this node will be "minimal_publisher2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("topic1", 1);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    std_msgs::Float64 input_float; //create a variable of type "Float64", 
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   
   ros::Rate naptime(1.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    input_float.data = 0.0;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        input_float.data = input_float.data + 0.001; //increment by 0.001 each iteration
        my_publisher_object.publish(input_float); // publish the value--of type Float64-- 
        //to the topic "topic1"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep(); 
    }
}

