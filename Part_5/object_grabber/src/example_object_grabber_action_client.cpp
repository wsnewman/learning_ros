// example_object_grabber_action_client: 
// wsn, August, 2016
// illustrates use of object_grabber action server called "objectGrabberActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>

XformUtils *g_formUtils;

Eigen::Affine3d gripper_affine_from_toyblock_pose(geometry_msgs::Pose object_pose) {
    Eigen::Affine3d object_affine, gripper_affine;
    //use XformUtils to convert from pose to affine:
    object_affine = g_formUtils->transformPoseToEigenAffine3d(object_pose);
    //cout << "object_affine origin: " << object_affine.translation().transpose() << endl;
    //cout << "object_affine R matrix: " << endl;
    //cout << object_affine.linear() << endl;

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
    //cout << "gripper_affine origin: " << gripper_affine.translation().transpose() << endl;
    //cout << "gripper_affine R matrix: " << endl;
    //cout << gripper_affine.linear() << endl;
    return gripper_affine;
}

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ",result->return_code);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_grabber_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    XformUtils xformUtils;
    g_formUtils = &xformUtils;
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("objectGrabberActionServer", true);
    geometry_msgs::PoseStamped toy_block_poseStamped,cylinder_poseStamped, desired_toolflange_poseStamped;
    //hard code an object pose; later, this will come from perception
    toy_block_poseStamped.header.frame_id = "torso";//set approach pose for toy block
    toy_block_poseStamped.pose.position.x = 0.5;
    toy_block_poseStamped.pose.position.y = -0.35;
    toy_block_poseStamped.pose.position.z = -0.105; //should be top of block; decide top surface vs object frame origin (in center)
    toy_block_poseStamped.pose.orientation.x = 0;
    toy_block_poseStamped.pose.orientation.y = 0;
    toy_block_poseStamped.pose.orientation.z = 0.842;
    toy_block_poseStamped.pose.orientation.w = 0.54;
    toy_block_poseStamped.header.stamp = ros::Time::now();

    cylinder_poseStamped.header.frame_id = "torso";//set approach pose for toy block
    cylinder_poseStamped.pose.position.x = 0.8;
    cylinder_poseStamped.pose.position.y = 0;
    cylinder_poseStamped.pose.position.z = -0.05; //should be center of upright cylinder
    cylinder_poseStamped.pose.orientation.x = 0;
    cylinder_poseStamped.pose.orientation.y = 0;
    cylinder_poseStamped.pose.orientation.z = 0.0;
    cylinder_poseStamped.pose.orientation.w = 1;
    cylinder_poseStamped.header.stamp = ros::Time::now();

    
    //need gripper-to-toolflange transform; use a transform listener
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 
    
     
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber::object_grabberResult object_grabber_result;

    
    //populate a viable object pose; actually, this is a Merry right-hand pose
   /* example from Merry: (right_hand w/rt torso?)
- Translation: [0.680, -0.205, 0.047]
- Rotation: in Quaternion [0.166, 0.684, 0.702, 0.109]
            in RPY (radian) [1.563, -0.084, 2.750]
            in RPY (degree) [89.537, -4.810, 157.546]
        */

    bool finished_before_timeout;   
    //stuff a goal message:
    object_grabber_goal.object_code = object_grabber::object_grabberGoal::TOY_BLOCK; //specify the object to be grabbed
    object_grabber_goal.object_frame = toy_block_poseStamped;
    ROS_INFO("attempt to grab toy block at object pose: ");
    xformUtils.printStampedPose(toy_block_poseStamped);
    ROS_INFO("sending goal: ");
        object_grabber_ac.sendGoal(object_grabber_goal,&objectGrabberDoneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }

    //try upright cylinder:
    object_grabber_goal.object_code = object_grabber::object_grabberGoal::COKE_CAN; //specify the object to be grabbed
    object_grabber_goal.object_frame = cylinder_poseStamped;
    ROS_INFO("attempt to grab upright cylinder at object pose: ");
    xformUtils.printStampedPose(cylinder_poseStamped);
    ROS_INFO("sending goal: ");
        object_grabber_ac.sendGoal(object_grabber_goal,&objectGrabberDoneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    return 0;
}

