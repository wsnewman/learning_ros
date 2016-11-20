// example_object_finder_action_client: 
// wsn, April, 2016
// illustrates use of object_finder action server called "objectFinderActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <object_manipulation_properties/object_ID_codes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>


geometry_msgs::PoseStamped g_perceived_object_pose;
ros::Publisher *g_pose_publisher;

int g_found_object_code;
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_finder_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("object_finder_action_service", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    object_finder::objectFinderGoal object_finder_goal;
    //object_finder::objectFinderResult object_finder_result;

    object_finder_goal.object_id = ObjectIdCodes::TABLE_SURFACE;
    object_finder_goal.known_surface_ht = false; //require find table height
    //object_finder_goal.object_id=object_finder::objectFinderGoal::COKE_CAN_UPRIGHT;
    //object_finder_goal.object_id=object_finder::objectFinderGoal::TOY_BLOCK;
    //object_finder_goal.known_surface_ht=true;
    //object_finder_goal.known_surface_ht=false; //require find table height
    //object_finder_goal.surface_ht = 0.05;
    double surface_height;
    ROS_INFO("sending goal: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        bool finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) {
                        ROS_INFO("surface-finder success");
                        surface_height = g_perceived_object_pose.pose.position.z; // table-top height, as found by object_finder 
                        ROS_INFO("found table ht = %f",surface_height);   }
    else {
        ROS_WARN("did not find table height; quitting:");
        return 1;
    }
    //if here, then find block using known table height:
    object_finder_goal.known_surface_ht = true;
    object_finder_goal.surface_ht = surface_height;
    ROS_INFO("using surface ht = %f",surface_height);        
    object_finder_goal.object_id=ObjectIdCodes::TOY_BLOCK_ID;
     ROS_INFO("sending goal to find TOY_BLOCK: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }       
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object!");
        return 0;
    }    
    else {
        ROS_WARN("object not found!:");
        return 1;
    }

}

