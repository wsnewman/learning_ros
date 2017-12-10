#ifndef XFORM_UTILS_H_
#define XFORM_UTILS_H_

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/TransformStamped.h>
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

class XformUtils {
public:
   Eigen::Affine3f transformTFToAffine3f(const tf::Transform &t);
   Eigen::Affine3d transformTFToAffine3d(const tf::Transform &t);
   double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
   geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi);
   tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
   geometry_msgs::PoseStamped get_pose_from_stamped_tf(tf::StampedTransform sTf);
   bool multiply_stamped_tfs(tf::StampedTransform A_stf,
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf);
   tf::StampedTransform stamped_transform_inverse(tf::StampedTransform sTf);
   geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e);
   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   
   
   Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
   Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::PoseStamped stPose); 
   Eigen::Affine3d transformStampedTfToEigenAffine3d(tf::StampedTransform sTf);   
   tf::StampedTransform convert_poseStamped_to_stampedTransform(geometry_msgs::PoseStamped stPose, std::string child_frame_id);  
   geometry_msgs::PoseStamped get_pose_stamped_from_odom(nav_msgs::Odometry odom);
   
   void test_stf(geometry_msgs::PoseStamped stPose);
   void printTf(tf::Transform tf);
   void printStampedTf(tf::StampedTransform sTf);
   void printStampedPose(geometry_msgs::PoseStamped stPose); 
   void printPose(geometry_msgs::Pose pose);
   //overload printPose to work with either Pose or PoseStamped:
   void printPose(geometry_msgs::PoseStamped stPose) {printStampedPose(stPose);}
   void printAffine(Eigen::Affine3d affine); //finish/test this fnc
};

#endif
