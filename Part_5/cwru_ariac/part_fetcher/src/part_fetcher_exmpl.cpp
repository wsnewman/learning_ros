#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include<part_fetcher/PartFetcherAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>

using namespace std;
XformUtils xformUtils; //type conversion utilities

geometry_msgs::PoseStamped g_object_pickup_poseStamped;
geometry_msgs::PoseStamped g_object_dropoff_poseStamped;
int g_object_ID;
bool g_received_order=false;

class PartFetcherActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<part_fetcher::PartFetcherAction> as_;
    
    // here are some message types to communicate with our client(s)
    part_fetcher::PartFetcherGoal goal_; // goal message, received from client
    part_fetcher::PartFetcherResult result_; // put results here, to be sent back to the client when done w/ goal
    part_fetcher::PartFetcherFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    PartFetcherActionServer(); //define the body of the constructor outside of class definition

    ~PartFetcherActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal);
};


PartFetcherActionServer::PartFetcherActionServer() :
   as_(nh_, "part_fetcher", boost::bind(&PartFetcherActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of PartFetcherActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}



void PartFetcherActionServer::executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal) {
    
     g_object_pickup_poseStamped = goal->object_frame;
     g_object_dropoff_poseStamped = goal->desired_frame;
     g_object_ID = goal->object_id;
     ROS_INFO("requested fetch of part_id = %d from location ",g_object_ID);
     xformUtils.printStampedPose(g_object_pickup_poseStamped);
     ROS_INFO("requested dropoff at pose: ");
     xformUtils.printStampedPose(g_object_dropoff_poseStamped);
     g_received_order=true; //set flag that new order has been received    
    
    result_.rtn_code=0; 
    as_.setSucceeded(result_); 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "part_fetcher_action_server"); // name this node 

    ROS_INFO("instantiating the part-fetcher action server: ");

    PartFetcherActionServer as_object; 
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
        if (g_received_order) {
         ROS_WARN("main: received new order; should act on it!");
         g_received_order=false;
        }
    }

    return 0;
}
