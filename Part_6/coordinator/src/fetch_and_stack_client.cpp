// fetch_and_stack_client: 
// wsn, October, 2016

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <coordinator/OpenLoopNavSvc.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>

bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code=0;
int g_object_finder_return_code=0;
int g_fdbk_count = 0;

geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
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
            g_object_grabber_return_code= result->object_grabber_return_code;
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

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle  

    coordinator::ManipTaskGoal goal;
    coordinator::OpenLoopNavSvc openLoopNavSvcMsg;
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_pose;
    move_base_msgs::MoveBaseGoal move_base_goal;
    XformUtils xform_utils; //instantiate an object of XformUtils
    
    geometry_msgs::PoseStamped pose_table2_approach, via_pose;
    //want to navigate to this pose to approach table2
    pose_table2_approach.header.frame_id="/map";
    pose_table2_approach.header.stamp = ros::Time::now();
    pose_table2_approach.pose.orientation.x=0.0;
    pose_table2_approach.pose.orientation.y=0.0;
    pose_table2_approach.pose.position.z=0.0;        
    pose_table2_approach.pose.position.x = -8.8;
    pose_table2_approach.pose.position.y = 0.18;
    pose_table2_approach.pose.orientation.z= -0.707; 
    pose_table2_approach.pose.orientation.w= 0.707;    
    via_pose = pose_table2_approach;
    via_pose.pose.position.x = 0.62;
    via_pose.pose.position.y = 6.1;    
    via_pose.pose.orientation.z = 0.707;
    via_pose.pose.orientation.w = 0.707;
    double table2_yaw_des = -1.57;
    double table2_y_des = -0.2;
    
    bool tferr=true;
    ROS_INFO("waiting for tf between map and base_link...");
    tf::StampedTransform tfBaseLinkWrtMap; 
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
                tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good; current pose is:");
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);    

    actionlib::SimpleActionClient<coordinator::ManipTaskAction> action_client("manip_task_action_service", true);
    ros::ServiceClient nav_move_client = nh.serviceClient<coordinator::OpenLoopNavSvc>("open_loop_nav_service");
    ros::ServiceClient nav_yaw_client = nh.serviceClient<coordinator::OpenLoopNavSvc>("open_loop_yaw_service");
    // attempt to connect to the server:
    ROS_INFO("waiting for the manipulation action server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = action_client.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigator_ac("move_base", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for move_base server: ");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to move_base action server"); // if here, then we connected to the server; 
     

    ROS_INFO("sending a goal: move arms to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move quitting");
        return 0;
    }
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
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
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
    
    //send command to acquire block:
    ROS_INFO("sending a goal: grab block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GRAB_OBJECT;
    goal.pickup_frame = g_result.object_pose;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
    //goal.perception_source= coordinator::ManipTaskGoal::BLIND_MANIP;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to grab block; quitting");
        return 0;
    }
    
    ROS_INFO("sending a goal: move arms to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move to pre-pose; quitting");
        return 0;
    }
    
    ROS_INFO("backing up");
    openLoopNavSvcMsg.request.move_distance= -1.0; // back up 1m
    nav_move_client.call(openLoopNavSvcMsg);
    ros::Duration(1.0).sleep(); //wait to settle down
    tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);    
    double yaw = xform_utils.convertPlanarQuat2Phi(current_pose.pose.orientation);
    ROS_INFO("yaw = %f",yaw);
    //desired yaw is 90 deg = 1.57 rad
    openLoopNavSvcMsg.request.move_distance= 1.57-yaw;
    nav_yaw_client.call(openLoopNavSvcMsg);
    tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);   
    ROS_INFO("navigating to via point");
    move_base_goal.target_pose = via_pose;
    navigator_ac.sendGoal(move_base_goal,&navigatorDoneCb);  
    bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }       
    ROS_INFO("navigating to table 2: ");
    move_base_goal.target_pose = pose_table2_approach;
    //try moving first to via point at:
    //(0.63, 6.1, 0) = (x,y,z); (0,0,0.707,0.707) = (x,y,z,w)    
    
    navigator_ac.sendGoal(move_base_goal,&navigatorDoneCb); 
   
    finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }    
    
    tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);    
    yaw = xform_utils.convertPlanarQuat2Phi(current_pose.pose.orientation);
    ROS_INFO("yaw = %f",yaw);
    //desired yaw is -90 deg = -1.57 rad
    openLoopNavSvcMsg.request.move_distance= -1.57-yaw;
    nav_yaw_client.call(openLoopNavSvcMsg);    
    //and move forward as well:
     tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose); 

    ROS_INFO("approaching table 2");
    //should be pointing in -y direction w/rt map; compute how much to creep up on table:
    openLoopNavSvcMsg.request.move_distance= -(table2_y_des  -current_pose.pose.position.y); 
    nav_move_client.call(openLoopNavSvcMsg);
    ros::Duration(1.0).sleep(); //wait to settle down
    tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);     
    
    //time to stack the block:
    //send vision goal to find block:
    ROS_INFO("sending a goal: find block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GET_PICKUP_POSE;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
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
    goal.dropoff_frame = g_object_pose; //frame per PCL perception
    goal.dropoff_frame.pose.position.z+=0.035; //set height to one block thickness higher
                                               // so new block will stack on prior block
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
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

