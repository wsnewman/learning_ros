//xform_pose.cpp: example use of xform_utils
// populate a hard-coded geometry_msgs::Pose, convert to Affine,
// construct a 2nd Affine derived from this Affine;
// convert the 2nd Affine back to a geometry_msgs::Pose
//wsn 8/16
#include<ros/ros.h>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_xform_utils"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    XformUtils xformUtils; //instantiate an object of XformUtils
    geometry_msgs::Pose object_pose, gripper_pose;
    geometry_msgs::PoseStamped gripper_pose_stamped;

    //define a pose with origin (x,y,z) = (0.5, -0.35, -0.155).
    object_pose.position.x = 0.5;
    object_pose.position.y = -0.35;
    object_pose.position.z = -0.155;
    //and orientation (x,y,z,w) = (0, 0, 0.842, 0.54)
    object_pose.orientation.x = 0;
    object_pose.orientation.y = 0;
    object_pose.orientation.z = 0.842;
    object_pose.orientation.w = 0.54;
    ROS_INFO("object pose: ");
    xformUtils.printPose(object_pose);
    Eigen::Affine3d object_affine, gripper_affine;
    //use XformUtils to convert from pose to affine:
    object_affine = xformUtils.transformPoseToEigenAffine3d(object_pose);
    cout << "object_affine origin: " << object_affine.translation().transpose() << endl;
    cout << "object_affine R matrix: " << endl;
    cout << object_affine.linear() << endl;

    //compute a gripper pose with z-axis anti-parallel to object z-axis,
    // and x-axis coincident with object x-axis
    Eigen::Vector3d x_axis, y_axis, z_axis;
    Eigen::Matrix3d R_object, R_gripper;
    R_object = object_affine.linear(); //get 3x3 matrix from affine
    x_axis = R_object.col(0); //extract the x axis
    z_axis = -R_object.col(2); //define gripper z axis antiparallel to object z-axis
    y_axis = z_axis.cross(x_axis); // construct a right-hand coordinate frame
    R_gripper.col(0) = x_axis; //populate orientation matrix from axis directions
    R_gripper.col(1) = y_axis;
    R_gripper.col(2) = z_axis;
    gripper_affine.linear() = R_gripper; //populate affine w/ orientation
    gripper_affine.translation() = object_affine.translation(); //and origin
    cout << "gripper_affine origin: " << gripper_affine.translation().transpose() << endl;
    cout << "gripper_affine R matrix: " << endl;
    cout << gripper_affine.linear() << endl;


    //use XformUtils fnc to convert from Affine to a pose
    gripper_pose = xformUtils.transformEigenAffine3dToPose(gripper_affine);
    gripper_pose_stamped.pose = gripper_pose;
    gripper_pose_stamped.header.stamp = ros::Time::now();
    gripper_pose_stamped.header.frame_id = "torso";
    ROS_INFO("desired gripper pose: ");
    xformUtils.printStampedPose(gripper_pose_stamped); //display the output
    
    //create a StampedTransform naming the gripper frame:
    geometry_msgs::PoseStamped test_pst;
    std::string child_frame = "generic_gripper_frame";
    cout<<"stf from stamped pose: "<<endl;
            
    //tf::StampedTransform gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(gripper_pose_stamped, child_frame);
    tf::StampedTransform gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(gripper_pose_stamped, "generic_gripper_frame");    
    xformUtils.printStampedTf(gripper_stf);

}
