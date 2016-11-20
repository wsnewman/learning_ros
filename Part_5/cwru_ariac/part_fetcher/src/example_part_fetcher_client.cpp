// example_part_fetcher_client: 
// wsn, Nov, 2016

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<part_fetcher/PartFetcherAction.h>

void set_example_object_frames(geometry_msgs::PoseStamped &object_poseStamped,
        geometry_msgs::PoseStamped &object_dropoff_poseStamped) {
    //hard code an object pose; later, this will come from perception
    //specify reference frame in which this pose is expressed:
    //will require that "system_ref_frame" is known to tf
    object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = -0.85;
    object_poseStamped.pose.position.y = 0.85;
    object_poseStamped.pose.position.z = 0.5622; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.0;
    object_poseStamped.pose.orientation.w = 1;
    object_poseStamped.header.stamp = ros::Time::now();

    object_dropoff_poseStamped = object_poseStamped; //specify desired drop-off pose of object
    object_dropoff_poseStamped.pose.position.x = 1;
    object_dropoff_poseStamped.pose.position.y = 0;
    object_dropoff_poseStamped.pose.position.z = 0.793; 
}


void doneCb(const actionlib::SimpleClientGoalState& state,
        const part_fetcher::PartFetcherResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result rtn_val = %d", result->rtn_code);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "part_fetcher_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle
    // here is a "goal" object compatible with the server, 
    part_fetcher::PartFetcherGoal goal;
    geometry_msgs::PoseStamped object_pickup_poseStamped;
    geometry_msgs::PoseStamped object_dropoff_poseStamped;
    //specify object pick-up and drop-off frames 
    set_example_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);
    
    actionlib::SimpleActionClient<part_fetcher::PartFetcherAction> action_client("part_fetcher", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

        // stuff a goal message:
        goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; 
        goal.object_frame = object_pickup_poseStamped;
        goal.desired_frame =  object_dropoff_poseStamped;       
       
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
       

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 0;
        } 


    return 0;
}

