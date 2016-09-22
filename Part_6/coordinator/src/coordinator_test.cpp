// coordinator: 
// wsn, Sept, 2016
// illustrates use of object_finder, object_grabber (need  navigator) action servers

//trigger this process with:
// to find toy block:
//rostopic pub action_codes std_msgs/Int32 100
/// to grab toy block:
// rostopic pub action_codes std_msgs/Int32 101

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <coordinator/CoordinatorSrv.h> 
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<coordinator/ManipTaskAction.h>


int g_status_code;
int g_action_code = 0;

class TaskActionServer {
private:
    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<coordinator::ManipTaskAction> as_;

    // here are some message types to communicate with our client(s)
    coordinator::ManipTaskGoal goal_; // goal message, received from client
    coordinator::ManipTaskResult result_; // put results here, to be sent back to the client when done w/ goal
    coordinator::ManipTaskFeedback feedback_; // for feedback 

public:
    TaskActionServer(); //define the body of the constructor outside of class definition

    ~TaskActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal);
};

TaskActionServer::TaskActionServer() :
as_(nh_, "manip_task_action", boost::bind(&TaskActionServer::executeCB, this, _1), false) {
    ROS_INFO("in constructor of TaskActionServer...");
    // do any other desired initializations here...specific to your implementation
    as_.start(); //start the server running
}

const int SUCCESS=100;
const int ABORTED=666;
void TaskActionServer::executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB: received manipulation task");
    ROS_INFO("object code is: %d", goal->object_code);
    ROS_INFO("perception_source is: %d", goal->perception_source);
    g_status_code = 0; //coordinator::ManipTaskFeedback::RECEIVED_NEW_TASK;
    g_action_code = 1; //start with perceptual processing
    //do work here: this is where your interesting code goes
    while ((g_status_code != SUCCESS)&&(g_status_code != ABORTED)) { //coordinator::ManipTaskResult::MANIP_SUCCESS) {
        feedback_.feedback_status = g_status_code;
        as_.publishFeedback(feedback_);
        //ros::Duration(0.1).sleep();
        ROS_INFO("executeCB: g_status_code = %d",g_status_code);
        // each iteration, check if cancellation has been ordered

        if (as_.isPreemptRequested()) {
            ROS_WARN("goal cancelled!");
            result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
            g_action_code = 0;
            g_status_code = 0;
            as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
            return; // done with callback
        }
        //here is where we step through states:
        switch (g_action_code) {

            case 1:  
                ROS_INFO("starting new task; should call object finder");
                g_status_code = 1; //
                ROS_INFO("executeCB: g_action_code, status_code = %d, %d",g_action_code,g_status_code);
                ros::Duration(2.0).sleep();
                g_action_code = 2;                
                break;

            case 2: // also do nothing...but maybe comment on status? set a watchdog?
                g_status_code = 2;
                ROS_INFO("executeCB: g_action_code, status_code = %d, %d",g_action_code,g_status_code);
                ros::Duration(2.0).sleep();
                g_action_code = 3;
                break;

            case 3:
                g_status_code = SUCCESS; //coordinator::ManipTaskResult::MANIP_SUCCESS; //code 0
                ROS_INFO("executeCB: g_action_code, status_code = %d, %d",g_action_code,g_status_code);
                g_action_code = 0; // back to waiting state--regardless
                break;
            default:
                ROS_WARN("executeCB: error--case not recognized");
                break;
        }        
    ros::Duration(0.5).sleep();    
        
    }
    ROS_INFO("executeCB: manip success; returning success");
        //if we survive to here, then the goal was successfully accomplished; inform the client
        result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
        as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
            g_action_code = 0;
            g_status_code = 0;
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    

    TaskActionServer taskActionServer; //create a task action server

    ROS_INFO("main going into loop");
    while (ros::ok()) {
        ros::spinOnce(); //NEED spins, or action server does not respond
        ros::Duration(0.1).sleep();
    }
    return 0;
}

