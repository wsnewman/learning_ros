// example_tf_listener.h header file //
// wsn; March, 2016
// include this file in "example_tf_listener.cpp"

#ifndef EXAMPLE_TF_LISTENER_H_
#define EXAMPLE_TF_LISTENER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
//#include <tf/LinearMath/QuadWord.h>


// define a class, including a constructor, member variables and member functions
class DemoTfListener
{
public:
    DemoTfListener(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
 
    geometry_msgs::PoseStamped get_pose_from_transform(tf::StampedTransform tf);
    
    bool multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf,tf::StampedTransform &C_stf);

    void printStampedTf(tf::StampedTransform sTf);    
    void printStampedPose(geometry_msgs::PoseStamped stPose);    
    void printTf(tf::Transform tf);
    
    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);     
    //illustrates a tf_listener in a class; this is somewhat more complex than creating a tf_listener in main()
    tf::TransformListener* tfListener_; 
  
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor        
}; 

#endif  
