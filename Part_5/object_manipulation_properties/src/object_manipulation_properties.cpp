#include <object_manipulation_properties/object_manipulation_properties.h>
using namespace std;
XformUtils xformUtils;
ObjectManipulationProperties::ObjectManipulationProperties(void) {
    ROS_INFO("in constructor of objectManipulationProperties");
    //could do something useful here, like read in data from a file...
}

bool ObjectManipulationProperties::get_object_info(int object_id, Eigen::Affine3d &grasp_transform, double &approach_dist, double &gripper_close_test) {
    bool got_info = false;
    Eigen::Vector3d object_origin_wrt_gripper_frame;
    //Eigen::Quaternion object_orientation_wrt_gripper_frame;
    Eigen::Matrix3d R_object_wrt_gripper;
    Eigen::Matrix3d R_gripper;
    Eigen::Vector3d x_axis, y_axis, z_axis;
   Eigen::Vector3d origin_object_wrt_gripper;    
   geometry_msgs::Pose object_pose_wrt_gripper;
    switch (object_id) {
        case TOY_BLOCK_ID:
            //set approach distance and gripper-closure test val: MAGIC NUMBERS
            // appropriate ONLY for this object with Baxter right-hand gripper in simu
            approach_dist = 0.05; //old float64 TOY_BLOCK_APPROACH_DIST = 0.05
            gripper_close_test = 83.0; //old float64 TOY_BLOCK_GRIPPER_CLOSE_TEST_VAL = 80.0
            //derive gripper approach pose from block pose:
            //compute a gripper pose with z-axis anti-parallel to object z-axis,
            // and x-axis coincident with object x-axis

            origin_object_wrt_gripper<<0,0,0;
            x_axis<<1,0,0; // make block x-axis parallel to gripper x-axis;
            z_axis<<0,0,-1; // gripper z axis antiparallel to block z axis;
            y_axis = z_axis.cross(x_axis); //consistent triad

            R_gripper.col(0) = x_axis; //populate orientation matrix from axis directions
            R_gripper.col(1) = y_axis;
            R_gripper.col(2) = z_axis;
            //gripper_affine is defined to have origin coincident w/ block-frame origin, but z-axis antiparallel to block z-axis
            // and x-axis parallel to block-frame x-axis
            grasp_transform.linear() = R_gripper; //populate affine w/ orientation
            grasp_transform.translation() = origin_object_wrt_gripper; //and origin          
            object_pose_wrt_gripper = xformUtils.transformEigenAffine3dToPose(grasp_transform);
            ROS_INFO("object pose w/rt gripper: ");
            cout<<"R_gripper: "<<endl;
            cout<<R_gripper<<endl;            
            xformUtils.printPose(object_pose_wrt_gripper);
            
            //try block x-axis anti-parallel:
            R_gripper.col(0)= -x_axis;
            R_gripper.col(1)= z_axis.cross(-x_axis);
            grasp_transform.linear() = R_gripper;

            object_pose_wrt_gripper = xformUtils.transformEigenAffine3dToPose(grasp_transform);
            ROS_INFO("object pose w/rt gripper, x-axis antiparallel: ");
            cout<<"R_gripper: "<<endl;
            cout<<R_gripper<<endl;            
            xformUtils.printPose(object_pose_wrt_gripper);            
            return true;
            break;

            //case SOMETHING_ELSE:  //add more object cases here!
            //...
            //return true;
            //break;
        default:
            ROS_WARN("object ID not recognized");
            return false;
            break;
    }
}

