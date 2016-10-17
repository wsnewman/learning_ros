// stack_block_client: 
// looks for a table, then a block on the table, then attempts to stack a grasped
// block on top of the perceived block
// This assumes the robot is already holding a block, and that another block is
// within reach and within view
// Useful as part of the fetch-and-stack sequence
// wsn, October, 2016

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_manipulation_properties.h>
#include <example_gazebo_set_state/SrvInt.h> // this message type is defined in the current package
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <fstream>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
bool g_goal_done = true;
int g_ntasks_done = 0;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code=0;
int g_object_finder_return_code=0;
int g_fdbk_count = 0;
int g_n_successes = 0;
int g_n_perception_failures = 0;
int g_n_pickup_failures =0;
int g_n_dropoff_failures=0;
int g_n_gripper_failures=0;
geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
coordinator::ManipTaskResult g_result;
//bool g_goal_active=false;
using namespace std;

void save_failure_data(void) {
  std::ofstream outfile;

  outfile.open("failures.txt", std::ios_base::app);
  outfile << "g_object_grabber_return_code: "<<g_object_grabber_return_code<<endl;
  if (g_object_grabber_return_code==object_grabber::object_grabberResult::GRIPPER_FAILURE) {
      outfile<<"gripper failure, object pose: "<<endl;
      outfile<<"object origin: (x,y,z) = ("<<g_object_pose.pose.position.x<<", "<<g_object_pose.pose.position.y<<", "
              <<g_object_pose.pose.position.z<<")"<<endl;
      outfile<<"orientation: (qx,qy,qz,qw) = ("<<g_object_pose.pose.orientation.x<<","
              <<g_object_pose.pose.orientation.y<<","
              <<g_object_pose.pose.orientation.z<<","
              <<g_object_pose.pose.orientation.w<<")"<<endl;
  }
  else {
      outfile<<"manipulation failure code "<<g_object_grabber_return_code<<endl;
      outfile<<"manip failed with desired flange pose: "<<endl;
       outfile<<"object origin: (x,y,z) = ("<<g_des_flange_pose_stamped_wrt_torso.pose.position.x<<", "
               <<g_des_flange_pose_stamped_wrt_torso.pose.position.y<<", "
               <<g_des_flange_pose_stamped_wrt_torso.pose.position.z<<")"<<endl;
      outfile<<"orientation: (qx,qy,qz,qw) = ("<<g_des_flange_pose_stamped_wrt_torso.pose.orientation.x<<","
              <<g_des_flange_pose_stamped_wrt_torso.pose.orientation.y<<","
              <<g_des_flange_pose_stamped_wrt_torso.pose.orientation.z<<","
              <<g_des_flange_pose_stamped_wrt_torso.pose.orientation.w<<")"<<endl;     
  }

  outfile.close();
}

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
            g_n_perception_failures++;
            g_object_finder_return_code = result->object_finder_return_code;
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_n_pickup_failures++;

            g_object_grabber_return_code= result->object_grabber_return_code;
            if (g_object_grabber_return_code==object_grabber::object_grabberResult::GRIPPER_FAILURE) {
                 g_n_gripper_failures++;
            }
            g_object_pose = result->object_pose;
            g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            //for failed pickup, save failure code and pose:
            save_failure_data();
            break;
        case coordinator::ManipTaskResult::FAILED_DROPOFF:
            ROS_WARN("returned FAILED_DROPOFF");
            g_n_dropoff_failures++;
            g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            save_failure_data();            
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
    ros::init(argc, argv, "dropoff_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle  

    ros::ServiceClient block_state_client = nh.serviceClient<example_gazebo_set_state::SrvInt>("set_block_state");
    example_gazebo_set_state::SrvInt block_state_srv;

    int n_attempts = 0;


    block_state_srv.request.request_int = 0; //refers to toy-block model #1

    coordinator::ManipTaskGoal goal;

   
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

      
    //send vision request to find table top:
    ROS_INFO("sending a goal: seeking table top");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::FIND_TABLE_SURFACE;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    
    //send vision goal to find block:
    ROS_INFO("sending a goal: find block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GET_PICKUP_POSE;
    goal.object_code= TOY_BLOCK_ID;
    goal.perception_source = coordinator::ManipTaskGoal::PCL_VISION;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to find block quitting");
        return 0;
    }
    g_object_pose = g_result.object_pose;
    ROS_INFO_STREAM("object origin: (x,y,z) = ("<<g_object_pose.pose.position.x<<", "<<g_object_pose.pose.position.y<<", "
              <<g_object_pose.pose.position.z<<")"<<endl);
    ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = ("<<g_object_pose.pose.orientation.x<<","
              <<g_object_pose.pose.orientation.y<<","
              <<g_object_pose.pose.orientation.z<<","
              <<g_object_pose.pose.orientation.w<<")"<<endl);    
    
    //send command to set grasped block on top of existing block:
    ROS_INFO("sending a goal: stack block");
    
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::DROPOFF_OBJECT;
    //goal.dropoff_frame = dropoff_pose_ = goal->dropoff_frame;
    goal.dropoff_frame = g_object_pose; //test--put it back where found it
    goal.dropoff_frame.pose.position.z+=0.035; //set height to one block thickness higher
    goal.object_code= TOY_BLOCK_ID;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to drop off block; quitting");
        return 0;
    }    
    
    return 0;
}

