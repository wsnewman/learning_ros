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
 
const double UPDATE_RATE=50.0; // for a 50Hz update rate
// define a class, including a constructor, member variables and member functions
class OdomTf
{
public:
    OdomTf(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor

    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);   
    tf::StampedTransform put_pose_in_transform(geometry_msgs::Pose pose);
    tf::StampedTransform stamped_transform_inverse(tf::StampedTransform stf);
    geometry_msgs::PoseStamped get_pose_from_transform(tf::StampedTransform tf);
    geometry_msgs::PoseStamped get_pose_stamped_from_odom(nav_msgs::Odometry odom);
    

    tf::StampedTransform stfBaseLinkWrtOdom_; //base link w/rt odom frame; get this from tf; 
    tf::StampedTransform stfOdomWrtMap_; //odom frame w/rt map frame; get this from tf, published by amcl
    tf::StampedTransform stfBaseLink_wrt_Map_; //base link w/rt map frame; compute this
                                               // and publish it on tf
    tf::StampedTransform stfAmclBaseLinkWrtMap_;  
    tf::StampedTransform stfEstBaseWrtMap_;
    
    geometry_msgs::PoseStamped base_link_wrt_odom_; //can extract this from 
    geometry_msgs::PoseStamped base_link_wrt_map_; //this is what we care about; need to compute it
                                                   // then publish it

    tf::StampedTransform tfLink2ToOdom_;  
    tf::StampedTransform stfBaseLinkWrtDriftyOdom_;
    tf::StampedTransform stfDriftyOdomWrtBase_;
    tf::StampedTransform stfDriftyOdomWrtMap_;
    
    tf::StampedTransform get_tfBaseLinkWrtDriftyOdom() { return stfBaseLinkWrtDriftyOdom_; } 
    
    bool multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf,tf::StampedTransform &C_stf);

    void printStampedTf(tf::StampedTransform sTf);    
    void printStampedPose(geometry_msgs::PoseStamped stPose);    
    void printTf(tf::Transform tf);

    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);     
    //illustrates a tf_listener in a class; this is somewhat more complex than creating a tf_listener in main()
    tf::TransformListener* tfListener_;   
    tf::TransformBroadcaster br_;
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber odom_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber amcl_subscriber_; // subscribe to amcl message containing estimated robot pose w/rt map frame
    void initializeSubscribers();
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_rcvd);


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
