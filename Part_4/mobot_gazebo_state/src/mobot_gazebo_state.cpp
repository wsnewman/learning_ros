#include <ros/ros.h> 
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>


geometry_msgs::Pose g_mobot_pose; //this is the pose of the robot in the world, according to Gazebo
ros::Publisher g_pose_publisher; 


void model_state_CB(const gazebo_msgs::ModelStates& model_states) 
{ 
  int n_models = model_states.name.size();
  int imodel;
  //ROS_INFO("there are %d models in the transmission",n_models);
  bool found_name=false;
  for (imodel=0;imodel<n_models;imodel++) {
    std::string model_name(model_states.name[imodel]); 
    if (model_name.compare("mobot")==0) {
      //ROS_INFO("found match: mobot is model %d",imodel);
      found_name=true;
      break;
    }
  }
  if(found_name) {
    g_mobot_pose= model_states.pose[imodel];
    g_pose_publisher.publish(g_mobot_pose);
    }
  else
    {
      ROS_WARN("state of mobot model not found");
    }
} 



int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_model_publisher");
    ros::NodeHandle nh;

    g_pose_publisher= nh.advertise<geometry_msgs::Pose>("gazebo_mobot_pose", 1); 
    
    ros::Subscriber state_sub = nh.subscribe("gazebo/model_states",1,model_state_CB); 
 
    ros::spin();
}
