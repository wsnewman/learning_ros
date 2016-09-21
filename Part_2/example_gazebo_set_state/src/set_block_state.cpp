//example pgm to set a model state in Gazebo, specific for toy blocks: block0 through blockN
// could also do w/ rosservice call gazebo/set_model_state
#include <ros/ros.h> //ALWAYS need to include this
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include <sstream>
#include <example_gazebo_set_state/SrvInt.h>

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

using namespace std;

double rand_of_range(double minval, double maxval) {
   double x_range = maxval-minval;
   double rval = ((double) rand())/((double) RAND_MAX);  
   double x;
   x = minval+rval*x_range;
   return x;
}


geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

bool g_trigger_new_request=false;
int  g_block_num;

bool callback(example_gazebo_set_state::SrvIntRequest& request, example_gazebo_set_state::SrvIntResponse& response)
{
   
   g_trigger_new_request=true;
   g_block_num = request.request_int;
    ROS_INFO("callback activated; blocknum requested = %d",g_block_num);
    
  return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "set_block_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    geometry_msgs::Quaternion quat;
     ros::ServiceServer service = nh.advertiseService("set_block_state", callback);
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

    model_state_srv_msg.request.model_state.twist.linear.x= 0.0; //2cm/sec
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 0.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";
    string block("block");
    string blockN;
    //hard code, or could prompt, or could have command-line arg here:
    while(ros::ok()) {
     if (g_trigger_new_request) {
       g_trigger_new_request=false; //reset trigger
    //cout<<"enter a block number: "; //later, replace this with a service
    //int ans;
    //cin>>ans;
    blockN = block + SSTR(g_block_num);
    cout<<"block name: "<<blockN<<endl;
    //ROS_INFO("tough--doing block0");
    model_state_srv_msg.request.model_state.model_name = blockN; //"block0";
    //generate random poses, height 0.8, x=[0.4,0.7], y=[-0.45, 0.2], yaw 0 to pi
    double x_min = 0.4;
    double x_max = 0.7;
    double y_min = -0.45;
    double y_max = 0.2;
    double yaw_min = 0.0;
    double yaw_max = 3.14;

    double x = rand_of_range(x_min,x_max);
    double y = rand_of_range(y_min,y_max);
    double yaw = rand_of_range(yaw_min,yaw_max);
    quat = convertPlanarPsi2Quaternion(yaw);

    model_state_srv_msg.request.model_state.pose.position.x = x;
    model_state_srv_msg.request.model_state.pose.position.y = y;
    model_state_srv_msg.request.model_state.pose.position.z = 0.8;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = quat.x;
    model_state_srv_msg.request.model_state.pose.orientation.y = quat.y;
    model_state_srv_msg.request.model_state.pose.orientation.z = quat.z;
    model_state_srv_msg.request.model_state.pose.orientation.w = quat.w;
    
   
    set_model_state_client.call(model_state_srv_msg);
        //make sure service call was successful
        bool result = model_state_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to set_model_state failed!");
        else
            ROS_INFO("service call was successful");
    }
  ros::spinOnce();
  ros::Duration(0.1).sleep();
}
        
}
