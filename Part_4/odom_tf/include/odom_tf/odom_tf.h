// odom_tf.h header file //
// wsn; March, 2016
// include this file in "odom_tf.cpp"

#ifndef ODOM_TF_H_
#define ODOM_TF_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
 #include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/transform_broadcaster.h>
#include <xform_utils/xform_utils.h>
 
//const double ODOM_TF_UPDATE_RATE=50.0; // for a 50Hz update rate
// define a class, including a constructor, member variables and member functions
class OdomTf
{
public:
    OdomTf(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    XformUtils xform_utils;
    tf::StampedTransform put_pose_in_transform(geometry_msgs::Pose pose);
    tf::StampedTransform get_tfBaseLinkWrtDriftyOdom() { return stfBaseLinkWrtDriftyOdom_; } 
   
    tf::StampedTransform stfBaseLinkWrtOdom_; //base link w/rt odom frame; get this from tf; 
    tf::StampedTransform stfOdomWrtMap_; //odom frame w/rt map frame; get this from tf, published by amcl
    tf::StampedTransform stfBaseLink_wrt_Map_; //base link w/rt map frame; compute this
                                              // and publish it on tf
    tf::StampedTransform stfAmclBaseLinkWrtMap_;  
    tf::StampedTransform stfEstBaseWrtMap_;
    
    geometry_msgs::PoseStamped base_link_wrt_odom_; 
    geometry_msgs::PoseStamped base_link_wrt_map_; 
                                                   

    tf::StampedTransform tfLink2ToOdom_;  
    tf::StampedTransform stfBaseLinkWrtDriftyOdom_;
    tf::StampedTransform stfDriftyOdomWrtBase_;
    tf::StampedTransform stfDriftyOdomWrtMap_;
    
    //illustrates a tf_listener in a class; this is somewhat more complex than creating a tf_listener in main()
    tf::TransformListener* tfListener_;   
    tf::TransformBroadcaster br_;
    bool odom_tf_ready_;
    bool odom_tf_is_ready() { return odom_tf_ready_; }
    bool odom_ready_;
    bool amcl_ready_;
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber odom_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber amcl_subscriber_; // subscribe to amcl message containing estimated robot pose w/rt map frame
    void initializeSubscribers();
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_rcvd);
    ros::Publisher pose_publisher_; // = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    geometry_msgs::PoseStamped estBasePoseWrtMap_;

    //state values from odometry; these will get filled in by odom callback      
    nav_msgs::Odometry current_odom_; // fill in these objects from callbacks
    geometry_msgs::Pose odom_pose_;   
    int odom_count_;
    //double odom_vel_;
    //double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_; 
    geometry_msgs::Quaternion amcl_quat_;    
    geometry_msgs::Pose amcl_pose_;
        
}; 

#endif  
