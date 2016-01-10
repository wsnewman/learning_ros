#include <ros/ros.h>
//next line requires a dependency on custom_msgs within package.xml
#include <custom_msgs/VecOfDoubles.h> //this is the message type we are testing 

int main(int argc, char **argv) {
    ros::init(argc, argv, "vector_publisher"); // name of this node 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<custom_msgs::VecOfDoubles>("vec_topic", 1);
    
    custom_msgs::VecOfDoubles vec_msg; //create an instance of this message type
    double counter=0; 
    ros::Rate naptime(1.0); //create a ros object from the ros “Rate” class; set 1Hz rate
   
    vec_msg.dbl_vec.resize(3); //manually resize it to hold 3 doubles
    //After setting the size, one can access elements of this array conventionally, e.g.
    vec_msg.dbl_vec[0]=1.414;    
    vec_msg.dbl_vec[1]=2.71828;    
    vec_msg.dbl_vec[2]=3.1416;
    
    //Alternatively, one can use the vector member function “push_back()” to append data to an existing array, e.g.:
    vec_msg.dbl_vec.push_back(counter); // this makes the vector longer, to hold additional data 
    while(ros::ok())  {
    counter+=1.0;
    vec_msg.dbl_vec.push_back(counter);
    my_publisher_object.publish(vec_msg);
    naptime.sleep(); 
    }
}   

