//wsn started 8/16: a library of useful transforms; add more here
#include <xform_utils/xform_utils.h>
using namespace std;

geometry_msgs::Pose XformUtils::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();
    //cout<<"affine R: "<<endl;
    //cout<<Re<<endl;
    
    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    //cout<<"q: "<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<endl;
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

geometry_msgs::PoseStamped XformUtils::transformEigenAffine3dToPoseStamped(Eigen::Affine3d e) {
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped poseStamped;
    pose = transformEigenAffine3dToPose(e);

    poseStamped.pose = pose;
    poseStamped.header.stamp = ros::Time::now();
    return poseStamped;
}

//alternative: also include the reference frame ID
geometry_msgs::PoseStamped XformUtils::transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string frame_id) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped = transformEigenAffine3dToPoseStamped(e);
  poseStamped.header.frame_id = frame_id;
  return poseStamped;
} 

Eigen::Affine3d XformUtils::transformPoseToEigenAffine3d(geometry_msgs::PoseStamped stPose) {
    Eigen::Affine3d affine;
    geometry_msgs::Pose pose = stPose.pose;
    Eigen::Vector3d Oe;
    //ROS_WARN("xformUtils: input pose:");
    //printPose(pose);
    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;
    
    
    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;
    affine.translation() = Oe;
    //printAffine(affine);
    return affine;
}

Eigen::Affine3d XformUtils::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;
    //ROS_WARN("xformUtils: input pose:");
    //printPose(pose);
    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;
    
    
    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;
    affine.translation() = Oe;
    //printAffine(affine);
    return affine;
}

   //convert a stamped TF to and Eigen::Affine3d; child and parent frame id's are lost
   Eigen::Affine3d XformUtils::transformStampedTfToEigenAffine3d(tf::StampedTransform sTf) {
       tf::Transform transform = get_tf_from_stamped_tf(sTf);
       Eigen::Affine3d affine = transformTFToAffine3d(transform);
       return affine;
   } 

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

//convert a Tf to an Affine3d; knowledge of named parent and child frames is lost
Eigen::Affine3d XformUtils::transformTFToAffine3d(const tf::Transform &t) {
    Eigen::Affine3d e;
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

geometry_msgs::Quaternion  XformUtils::convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
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

//tf::StampedTransform convert_poseStamped_to_stampedTransform(geometry_msgs::PoseStamped stPose, std::string child_frame_id); 
tf::StampedTransform XformUtils::convert_poseStamped_to_stampedTransform(geometry_msgs::PoseStamped stPose, std::string child_frame_id) {

 tf::Transform transform;
 geometry_msgs::Pose pose = stPose.pose;

 geometry_msgs::Point position = pose.position;
 geometry_msgs::Quaternion orientation = pose.orientation;
 transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
 //cout<<"reference frame: "<<stPose.header.frame_id<<endl;
 //printStampedPose(stPose);
 transform.setRotation( tf::Quaternion( orientation.x, orientation.y, orientation.z, orientation.w) );
 tf::StampedTransform stTransform(transform, stPose.header.stamp, stPose.header.frame_id,child_frame_id);
 return stTransform;
}


void XformUtils::test_stf(geometry_msgs::PoseStamped stPose) {
    printStampedPose(stPose);
}



//compute C_stf = A_stf*B_stf; 
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

//fnc to print out a pose:
void XformUtils::printPose(geometry_msgs::Pose pose) {
    ROS_INFO_STREAM("origin: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << endl);
    ROS_INFO_STREAM("quaternion: " << pose.orientation.x << ", " << pose.orientation.y << ", "
            << pose.orientation.z << ", " << pose.orientation.w << endl);    
}
//fnc to print components of a stamped pose

void XformUtils::printStampedPose(geometry_msgs::PoseStamped stPose) {
    ROS_INFO_STREAM("frame id = " << stPose.header.frame_id << endl);
    ROS_INFO_STREAM("origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z << endl);
    ROS_INFO_STREAM("quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w << endl);
}

void XformUtils::printAffine(Eigen::Affine3d affine) {
    ROS_INFO_STREAM("origin: "<<affine.translation().transpose()<<endl);
    Eigen::Matrix3d R;
    R = affine.linear();
    //Eigen::Vector3d x_vec,y_vec,z_vec;
    //x_vec = R.col(0);
    //y_vec = R.col(1);
    //z_vec = R.col(2);
    ROS_INFO_STREAM("\n"<<R);
}
