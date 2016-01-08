// minimal_controller node: 
// wsn example node that both subscribes and publishes--counterpart to minimal_simulator 
// subscribes to "velocity" and publishes "force_cmd" 
// subscribes to "vel_cmd" 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
//global variables for callback functions to populate for use in main program 
std_msgs::Float64 g_velocity;
std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 g_force; // this one does not need to be global... 

void myCallbackVelocity(const std_msgs::Float64& message_holder) {
    // check for data on topic "velocity" 
    ROS_INFO("received velocity value is: %f", message_holder.data);
    g_velocity.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

void myCallbackVelCmd(const std_msgs::Float64& message_holder) {
    // check for data on topic "vel_cmd" 
    ROS_INFO("received velocity command value is: %f", message_holder.data);
    g_vel_cmd.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_controller"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_controller" 
    ros::NodeHandle nh; // node handle 
    //create 2 subscribers: one for state sensing (velocity) and one for velocity commands 
    ros::Subscriber my_subscriber_object1 = nh.subscribe("velocity", 1, myCallbackVelocity);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("vel_cmd", 1, myCallbackVelCmd);
    //publish a force command computed by this controller; 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("force_cmd", 1);
    double Kv = 1.0; // velocity feedback gain 
    double dt_controller = 0.1; //specify 10Hz controller sample rate (pretty slow, but 
    //illustrative) 
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 
    g_velocity.data = 0.0; //initialize velocity to zero 
    g_force.data = 0.0; // initialize force to 0; will get updated by callback 
    g_vel_cmd.data = 0.0; // init velocity command to zero 
    double vel_err = 0.0; // velocity error 
    // enter the main loop: get velocity state and velocity commands 
    // compute command force to get system velocity to match velocity command 
    // publish this force for use by the complementary simulator 
    while (ros::ok()) {
        vel_err = g_vel_cmd.data - g_velocity.data; // compute error btwn desired and actual 
        //velocities 
        g_force.data = Kv*vel_err; //proportional-only velocity-error feedback defines commanded 
        //force 
        my_publisher_object.publish(g_force); // publish the control effort computed by this 
        //controller 
        ROS_INFO("force command = %f", g_force.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 
