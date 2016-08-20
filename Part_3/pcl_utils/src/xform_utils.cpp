//wsn started 8/16--finish making this into a library of useful transforms
#include <xform_utils/xform_utils.h>
using namespace std;
/*
#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/TransformStamped.h>

class XformUtils {
   Eigen::Affine3f transformTFToAffine3f(const tf::Transform &t);
   double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
   tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
   geometry_msgs::PoseStamped get_pose_from_stamped_tf(tf::StampedTransform sTf);
   bool multiply_stamped_tfs(tf::StampedTransform A_stf,
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf);
   tf::StampedTransform stamped_transform_inverse(tf::StampedTransform sTf);
   
};
*/

Eigen::Affine3f XformUtils::transformTFToAffine3f(const tf::Transform &t) {
    Eigen::Affine3f e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

double XformUtils::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

tf::Transform XformUtils::get_tf_from_stamped_tf(tf::StampedTransform sTf) {
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}

geometry_msgs::PoseStamped XformUtils::get_pose_from_stamped_tf(tf::StampedTransform tf) {
    //clumsy conversions--points, vectors and quaternions are different data types in tf vs geometry_msgs
    geometry_msgs::PoseStamped stPose;
    geometry_msgs::Quaternion quat; //geometry_msgs object for quaternion
    tf::Quaternion tfQuat; // tf library object for quaternion
    tfQuat = tf.getRotation(); // member fnc to extract the quaternion from a transform
    quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
    quat.y = tfQuat.y();
    quat.z = tfQuat.z();
    quat.w = tfQuat.w();
    stPose.pose.orientation = quat; //set the orientation of our PoseStamped object from result

    // now do the same for the origin--equivalently, vector from parent to child frame 
    tf::Vector3 tfVec; //tf-library type
    geometry_msgs::Point pt; //equivalent geometry_msgs type
    tfVec = tf.getOrigin(); // extract the vector from parent to child from transform
    pt.x = tfVec.getX(); //copy the components into geometry_msgs type
    pt.y = tfVec.getY();
    pt.z = tfVec.getZ();
    stPose.pose.position = pt; //and use this compatible type to set the position of the PoseStamped
    stPose.header.frame_id = tf.frame_id_; //the pose is expressed w/rt this reference frame
    stPose.header.stamp = tf.stamp_; // preserve the time stamp of the original transform
    return stPose;
}

void XformUtils::printTf(tf::Transform tf) {
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tf.getOrigin();
    ROS_INFO_STREAM("vector from reference frame to child frame: " << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl);
    tfR = tf.getBasis();
    ROS_INFO_STREAM("orientation of child frame w/rt reference frame: " << endl);
    tfVec = tfR.getRow(0);
    ROS_INFO_STREAM(tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl);
    tfVec = tfR.getRow(1);
    ROS_INFO_STREAM(tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl);
    tfVec = tfR.getRow(2);
    ROS_INFO_STREAM(tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl);
    quat = tf.getRotation();
    ROS_INFO_STREAM("quaternion: " << quat.x() << ", " << quat.y() << ", "
            << quat.z() << ", " << quat.w() << endl);
}

bool XformUtils::multiply_stamped_tfs(tf::StampedTransform A_stf,
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf) {

    //long-winded approach:
    //std::string str1(A_stf.child_frame_id_); //want to compare strings to check consistency
    //std::string str2(B_stf.frame_id_);
    //if (str1.compare(str2) != 0) { //SHOULD get that child frame of A is parent frame of B
    //more compact approach:
    if (A_stf.child_frame_id_.compare(B_stf.frame_id_) != 0) {    
        std::cout << "can't multiply transforms; mismatched frames" << endl;
	std::cout << A_stf.child_frame_id_ << " is not " << B_stf.frame_id_ << '\n';
        return false;
    }
    //if here, the named frames are logically consistent
        tf::Transform C = A_stf*B_stf; //multiplication is defined for transforms
	C_stf.setData(C);
	C_stf.frame_id_ = A_stf.frame_id_;
	C_stf.child_frame_id_ = B_stf.child_frame_id_;
	C_stf.stamp_ = ros::Time::now();
 
    //long-winded approach, equivalent to above:
    /*
    tf::Transform A, B; //simple transforms--not stamped
        
    A = get_tf_from_stamped_tf(A_stf); // get the transform from the stamped transform
    B = get_tf_from_stamped_tf(B_stf);
    C = A*B; //multiplication is defined for transforms 
    C_stf.frame_id_ = A_stf.frame_id_; //assign appropriate parent and child frames to result
    C_stf.child_frame_id_ = B_stf.child_frame_id_;
    C_stf.setOrigin(C.getOrigin()); //populate the origin and orientation of the result
    C_stf.setBasis(C.getBasis());
    C_stf.stamp_ = ros::Time::now(); //assign the time stamp to current time; 
     * */
    // alternatively, could assign this to the OLDER of A or B transforms
    return true; //if got here, the multiplication is valid
}

tf::StampedTransform XformUtils::stamped_transform_inverse(tf::StampedTransform stf) {
    // instantiate stamped transform with constructor args
    //note: child_frame and frame_id are reversed, to correspond to inverted transform
    tf::StampedTransform stf_inv(stf.inverse(), stf.stamp_, stf.child_frame_id_, stf.frame_id_);
    /* long-winded equivalent:
    tf::StampedTransform stf_inv;    
    tf::Transform tf = get_tf_from_stamped_tf(stf);
    tf::Transform tf_inv = tf.inverse();
    
    stf_inv.stamp_ = stf.stamp_;
    stf_inv.frame_id_ = stf.child_frame_id_;
    stf_inv.child_frame_id_ = stf.frame_id_;
    stf_inv.setOrigin(tf_inv.getOrigin());
    stf_inv.setBasis(tf_inv.getBasis());
     * */
    return stf_inv;
}

void XformUtils::printStampedTf(tf::StampedTransform sTf) {
    tf::Transform tf;
    ROS_INFO_STREAM("frame_id: " << sTf.frame_id_ << endl);
    ROS_INFO_STREAM("child_frame_id: " << sTf.child_frame_id_ << endl);
    tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf  
    printTf(tf); //and print its components      
}

//fnc to print components of a stamped pose

void XformUtils::printStampedPose(geometry_msgs::PoseStamped stPose) {
    ROS_INFO_STREAM("frame id = " << stPose.header.frame_id << endl);
    ROS_INFO_STREAM("origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z << endl);
    ROS_INFO_STREAM("quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w << endl);
}


