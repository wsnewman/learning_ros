//example pgm to set a model state in Gazebo
// could also do w/ rosservice call gazebo/set_model_state
#include <ros/ros.h> //ALWAYS need to include this
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "init_model_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("waiting for set_model_state service");
      half_sec.sleep();
    }
    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client = 
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    gazebo_msgs::SetModelState model_state_srv_msg;
    
    //hard code, or could prompt, or could have command-line arg here:
    model_state_srv_msg.request.model_state.model_name = "rect_prism";
    model_state_srv_msg.request.model_state.pose.position.x = 0.0;
    model_state_srv_msg.request.model_state.pose.position.y = 0.0;
    model_state_srv_msg.request.model_state.pose.position.z = 5.0;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.y = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.z = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.w = 1.0;
    
    
    model_state_srv_msg.request.model_state.twist.linear.x= 0.02; //2cm/sec
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 1.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";

    set_model_state_client.call(model_state_srv_msg);
        //make sure service call was successful
        bool result = model_state_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to set_model_state failed!");
        else
            ROS_INFO("Done");
        
}
