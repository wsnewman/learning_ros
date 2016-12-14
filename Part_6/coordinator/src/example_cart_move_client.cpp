// example_cart_move_client: 
// wsn, December, 2016
//test client demonstrate how to command gripper to move to a specified destination

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
//#include <object_manipulation_properties/object_manipulation_properties.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code = 0;
int g_object_finder_return_code = 0;
int g_fdbk_count = 0;


geometry_msgs::PoseStamped g_object_pose;
coordinator::ManipTaskResult g_result;

using namespace std;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const coordinator::ManipTaskResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_goal_done = true;
    g_result = *result;
    g_callback_status = result->manip_return_code;

    switch (g_callback_status) {
        case coordinator::ManipTaskResult::MANIP_SUCCESS:
            ROS_INFO("returned MANIP_SUCCESS");

            break;

        case coordinator::ManipTaskResult::FAILED_PERCEPTION:
            ROS_WARN("returned FAILED_PERCEPTION");
            g_object_finder_return_code = result->object_finder_return_code;
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_object_grabber_return_code = result->object_grabber_return_code;
            g_object_pose = result->object_pose;
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            break;
        case coordinator::ManipTaskResult::FAILED_DROPOFF:
            ROS_WARN("returned FAILED_DROPOFF");
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;          
            break;
    }
}

//optional feedback; output has been suppressed (commented out) below

void feedbackCb(const coordinator::ManipTaskFeedbackConstPtr& fdbk_msg) {
    g_fdbk_count++;
    if (g_fdbk_count > 1000) { //slow down the feedback publications
        g_fdbk_count = 0;
        //suppress this feedback output
        //ROS_INFO("feedback status = %d", fdbk_msg->feedback_status);
    }
    //g_fdbk = fdbk_msg->feedback_status; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but possibly useful for diagnostics

void activeCb() {
    ROS_INFO("Goal just went active");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cart_move_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle  

    coordinator::ManipTaskGoal goal;
    
    //define a dropoff pose; 
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.header.frame_id = "torso";
    desired_pose.pose.position.x = 0.5;
    desired_pose.pose.position.y = 0; //-0.35;
    desired_pose.pose.position.z = -0.12;
    desired_pose.pose.orientation.x = 0.707;
    desired_pose.pose.orientation.y = 0.707;
    desired_pose.pose.orientation.z = 0;
    desired_pose.pose.orientation.w = 0;
    desired_pose.header.stamp = ros::Time::now();    

    actionlib::SimpleActionClient<coordinator::ManipTaskAction> action_client("manip_task_action_service", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = action_client.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;



            //send command to move gripper to destination pose: 
            ROS_INFO("sending a goal: move gripper to hard-coded destination");
            g_goal_done = false;
            goal.action_code = coordinator::ManipTaskGoal::CART_MOVE_TO_GRIPPER_POSE;
            goal.gripper_goal_frame = desired_pose;
            action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
            if (g_callback_status != coordinator::ManipTaskResult::MANIP_SUCCESS) {
                ROS_ERROR("failed to perform Cartesian move; ");
            }
        return 0;
}

