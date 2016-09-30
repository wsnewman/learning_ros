// coordinator_action_client2: 
// wsn, September, 2016

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_manipulation_properties.h>
#include <example_gazebo_set_state/SrvInt.h> // this message type is defined in the current package
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

bool g_goal_done = true;
int g_ntasks_done = 0;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_fdbk_count = 0;
int g_n_successes = 0;
int g_n_perception_failures = 0;
int g_n_pickup_failures =0;
//bool g_goal_active=false;
using namespace std;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const coordinator::ManipTaskResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_goal_done = true;

    g_callback_status = result->manip_return_code;
    switch (g_callback_status) {
        case coordinator::ManipTaskResult::MANIP_SUCCESS:
            ROS_INFO("returned MANIP_SUCCESS");
             //g_n_successes++; //note: this will count pre-pose and find table-top
            break;
        case coordinator::ManipTaskResult::FAILED_PERCEPTION:
            ROS_WARN("returned FAILED_PERCEPTION");
            g_n_perception_failures++;
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP_PLAN:
            ROS_WARN("returned FAILED_PICKUP_PLAN");
            break;
        case coordinator::ManipTaskResult::FAILED_DROPOFF_PLAN:
            ROS_WARN("returned FAILED_DROPOFF_PLAN");
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_n_pickup_failures++;
            break;
        case coordinator::ManipTaskResult::DROPPED_OBJECT:
            ROS_WARN("returned DROPPED_OBJECT");
            break;
    }
    //ROS_INFO("return status is %d ", g_callback_status);
}

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
    ros::init(argc, argv, "task_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle  
    //rosservice call set_block_state 5
    //ros::Publisher block_pose_publisher = nh.advertise<std_msgs::Float64>("topic1", 1);
    ros::ServiceClient block_state_client = nh.serviceClient<example_gazebo_set_state::SrvInt>("set_block_state");
    example_gazebo_set_state::SrvInt block_state_srv;
    //the following is just to keep the robot from slipping...
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("waiting for set_model_state service");
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client = 
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    gazebo_msgs::SetModelState model_state_srv_msg;
    model_state_srv_msg.request.model_state.model_name = "baxter_on_mobot";
    model_state_srv_msg.request.model_state.twist.linear.x= 0.0; //2cm/sec
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 0.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";
    model_state_srv_msg.request.model_state.pose.position.x = 0;
    model_state_srv_msg.request.model_state.pose.position.y = 0;
    model_state_srv_msg.request.model_state.pose.position.z = 0.0;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = 0;
    model_state_srv_msg.request.model_state.pose.orientation.y = 0;
    model_state_srv_msg.request.model_state.pose.orientation.z = 0;
    model_state_srv_msg.request.model_state.pose.orientation.w = 1;
    //do the following periodically to reset robot pose:
    set_model_state_client.call(model_state_srv_msg);    
    ros::Duration(2.0).sleep(); // wait for repositioning
    
    int n_attempts = 0;
    //int n_successes = 0;

    block_state_srv.request.request_int = 1; //refers to toy-block model #1

    coordinator::ManipTaskGoal goal;

    goal.dropoff_frame.header.frame_id = "torso";
    goal.dropoff_frame.pose.position.x = 0.5;
    goal.dropoff_frame.pose.position.y = -0.35;
    goal.dropoff_frame.pose.position.z = -0.145;
    goal.dropoff_frame.pose.orientation.x = 0;
    goal.dropoff_frame.pose.orientation.y = 0;
    goal.dropoff_frame.pose.orientation.z = 0.842;
    goal.dropoff_frame.pose.orientation.w = 0.54;
    goal.dropoff_frame.header.stamp = ros::Time::now();
    goal.pickup_frame = goal.dropoff_frame;
    goal.pickup_frame.pose.position.y = -0.5; //drop block here

    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
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


    ROS_INFO("sending a goal: move to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }

    //send vision request to find table top:
    ROS_INFO("sending a goal: seeking table top");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::FIND_TABLE_SURFACE;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }


    while (ros::ok()) { //manipulation test loop--keep retrying
        //send a manipulation code, including vision, grasp and drop-off
        g_goal_done = false;
        n_attempts++;

        goal.object_code = TOY_BLOCK_ID; // from object_manipulation_properties; //coordinator::ManipTaskGoal::TOY_BLOCK;
        goal.action_code = coordinator::ManipTaskGoal::MANIP_OBJECT;
        //goal.perception_source= coordinator::ManipTaskGoal::BLIND_MANIP;
        goal.perception_source = coordinator::ManipTaskGoal::PCL_VISION;

        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        while (!g_goal_done) {
            ros::Duration(0.1).sleep();
        }
            g_ntasks_done++;
            
        if (g_callback_status == coordinator::ManipTaskResult::MANIP_SUCCESS) {
            ROS_WARN("returned MANIP_SUCCESS");
            g_n_successes++;
        } else {
            ROS_ERROR("failure: code %d; going back to pre-pose", g_callback_status);
            g_goal_done = false;
            goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
            action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            while (!g_goal_done) {
                ros::Duration(0.1).sleep();
            }
        }
        ROS_WARN("got %d successes in %d tries", g_n_successes, n_attempts);
        ROS_WARN("failed pickups: %d; failed perception %d: ",g_n_pickup_failures,g_n_perception_failures);
        ROS_INFO("setting up another block");
        set_model_state_client.call(model_state_srv_msg);       
        block_state_client.call(block_state_srv);
        ros::Duration(2.0).sleep(); //wait for block to show up
        //ROS_INFO("callback reports goal is done; enter 1 to run again:");
        //int ans;
        //cin>>ans;
    }
    return 0;
}

