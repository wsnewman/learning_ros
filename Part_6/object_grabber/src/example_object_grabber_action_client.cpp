// example_object_grabber_action_client: 
// wsn, April, 2016
// illustrates use of object_grabber action server called "objectGrabberActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ",result->return_code);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_grabber_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("objectGrabberActionServer", true);
    
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
    geometry_msgs::PoseStamped perceived_object_pose;
    
    //populate a viable object pose; actually, this is a Merry right-hand pose
   /* example from Merry: (right_hand w/rt torso?)
- Translation: [0.680, -0.205, 0.047]
- Rotation: in Quaternion [0.166, 0.684, 0.702, 0.109]
            in RPY (radian) [1.563, -0.084, 2.750]
            in RPY (degree) [89.537, -4.810, 157.546]
        */
    perceived_object_pose.header.frame_id = "torso";
    perceived_object_pose.pose.position.x = 0.680;
    perceived_object_pose.pose.position.y = -0.205;
    perceived_object_pose.pose.position.z = 0.047;
    perceived_object_pose.pose.orientation.x = 0.166;
    perceived_object_pose.pose.orientation.y = 0.648;
    perceived_object_pose.pose.orientation.z = 0.702;
    perceived_object_pose.pose.orientation.w = 0.109;
    
    //stuff a goal message:
    object_grabber_goal.object_code = object_grabber::object_grabberGoal::COKE_CAN; //specify the object to be grabbed
    object_grabber_goal.object_frame = perceived_object_pose;
    ROS_INFO("sending goal: ");
        object_grabber_ac.sendGoal(object_grabber_goal,&objectGrabberDoneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    return 0;
}

