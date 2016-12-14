// command_bundler: handles vision, manipulation and navigation commands piecewise 
// wsn, Oct, 2016

  
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
//#include <coordinator/CoordinatorSrv.h> 
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<coordinator/ManipTaskAction.h>
//#include <object_manipulation_properties/object_manipulation_properties.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>

class TaskActionServer {
private:
    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<coordinator::ManipTaskAction> as_;

    // here are some message types to communicate with our client(s)
    coordinator::ManipTaskGoal goal_; // goal message, received from client
    coordinator::ManipTaskResult result_; // put results here, to be sent back to the client when done w/ goal
    coordinator::ManipTaskFeedback feedback_; // for feedback 
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac_;
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac_;
    

    object_finder::objectFinderGoal object_finder_goal_;
    object_grabber::object_grabberGoal object_grabber_goal_;
    void objectGrabberDoneCb_(const actionlib::SimpleClientGoalState& state,
            const object_grabber::object_grabberResultConstPtr& result);
    void objectFinderDoneCb_(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result);
    int object_grabber_return_code_; //feedback status from object grabber
    int found_object_code_; //feedback status from object finder

    //the following items are elements of the goal message
    geometry_msgs::PoseStamped pickup_pose_;
    geometry_msgs::PoseStamped dropoff_pose_;
    int goal_action_code_, object_code_, perception_source_;
    int vision_object_code_; //SHOULD be reconciled with ManipTask object codes
    double surface_height_; // table-top height, as found by object_finder
    bool found_surface_height_;

    //the following are used for logic in executeCB
    bool working_on_task_; //true as long as goal is still in progress
    int status_code_; //values to be published as feedback during action service
    int action_code_, pickup_action_code_, dropoff_action_code_;
    ros::Publisher pose_publisher_;

public:
    TaskActionServer(); //define the body of the constructor outside of class definition

    ~TaskActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal);
};

TaskActionServer::TaskActionServer() :
as_(nh_, "manip_task_action_service", boost::bind(&TaskActionServer::executeCB, this, _1), false),
object_finder_ac_("object_finder_action_service", true),
object_grabber_ac_("object_grabber_action_service", true) {
    ROS_INFO("in constructor of TaskActionServer...");
    // do any other desired initializations here...specific to your implementation
    as_.start(); //start the server running

    action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
    status_code_ = coordinator::ManipTaskFeedback::NO_CURRENT_TASK;
    working_on_task_ = false;

    //connect to the object_grabber server
    ROS_INFO("waiting for object-grabber action server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 

    ROS_INFO("attempting to connect to object-finder action server");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server");


    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    found_surface_height_ = false;
}

void TaskActionServer::objectFinderDoneCb_(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    //found_object_code_ = result->found_object_code;
    ROS_INFO("got object code response = %d; ", result->found_object_code);
    if (result->found_object_code == object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    } else if (result->found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
        pickup_pose_ = result->object_pose;
        result_.object_pose = pickup_pose_;
        ROS_INFO("got pose x,y,z = %f, %f, %f", pickup_pose_.pose.position.x,
                pickup_pose_.pose.position.y,
                pickup_pose_.pose.position.z);
        pose_publisher_.publish(pickup_pose_);
    } else {
        ROS_WARN("object not found!");
    }
    //put this last--possible race condition with main!
    // pass this code back to the client
    result_.object_finder_return_code = found_object_code_;
    found_object_code_ = result->found_object_code;
}

void TaskActionServer::objectGrabberDoneCb_(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
        
    ROS_INFO("got result output = %d; ", result->return_code);

    //result_.des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
    
    //pass return code back to the client    
    result_.object_grabber_return_code = object_grabber_return_code_;
    object_grabber_return_code_ = result->return_code; //put this last, to avoid race condition
}


void TaskActionServer::executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB: received manipulation task");

    goal_action_code_ = goal->action_code; //verbatim from received goal
    action_code_ = goal->action_code; //init: this value changes as state machine advances through steps
    ROS_INFO("requested action code is: %d", goal_action_code_);
 
    if (goal_action_code_ == coordinator::ManipTaskGoal::DROPOFF_OBJECT) {
        object_code_ = goal->object_code; //what type of object is this?   
    } else if (goal_action_code_ == coordinator::ManipTaskGoal::GRAB_OBJECT) {
        object_code_ = goal->object_code; //what type of object is this?     
        ROS_INFO("object code is: %d", object_code_);
 
        if (goal->perception_source == coordinator::ManipTaskGoal::BLIND_MANIP) {
            ROS_INFO("blind manipulation; using provided pick-up pose");
            pickup_pose_ = goal->pickup_frame;
        }
    }
       else if (goal_action_code_ == coordinator::ManipTaskGoal::STRADDLE_OBJECT) {
        object_code_ = goal->object_code; //what type of object is this?     
        ROS_INFO("object code is: %d", object_code_);
        pickup_pose_ = goal->pickup_frame;       
    } else if (goal_action_code_ == coordinator::ManipTaskGoal::GET_PICKUP_POSE) {
        ROS_INFO("object code is: %d", object_code_);
        ROS_INFO("perception_source is: %d", goal->perception_source);
        object_code_ = goal->object_code; //what type of object is this?
        perception_source_ = goal->perception_source; //name sensor or provide coords
        vision_object_code_ = object_code_; //same code
    }

    status_code_ = coordinator::ManipTaskFeedback::RECEIVED_NEW_TASK; //coordinator::ManipTaskFeedback::RECEIVED_NEW_TASK;
    working_on_task_ = true;
    //do work here
    while (working_on_task_) { //coordinator::ManipTaskResult::MANIP_SUCCESS) {
        feedback_.feedback_status = status_code_;
        as_.publishFeedback(feedback_);
        //ROS_INFO("executeCB: status_code = %d", status_code_);
        // each iteration, check if cancellation has been ordered

        if (as_.isPreemptRequested()) {
            ROS_WARN("goal cancelled!");
            result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
            action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
            status_code_ = coordinator::ManipTaskFeedback::ABORTED;
            working_on_task_ = false;
            as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
            return; // done with callback
        }
        //here is where we step through states:
        switch (action_code_) {
            case coordinator::ManipTaskGoal::FIND_TABLE_SURFACE:
                ROS_INFO("serving request to find table surface");
                found_object_code_ = object_finder::objectFinderResult::OBJECT_FINDER_BUSY;
                object_finder_goal_.object_id = ObjectIdCodes::TABLE_SURFACE;
                //vision_object_code_;
                object_finder_goal_.known_surface_ht = false; //require find table height
                //object_finder_goal_.surface_ht = 0.05; //this is ignored for known_surface_ht=false
                object_finder_ac_.sendGoal(object_finder_goal_,
                        boost::bind(&TaskActionServer::objectFinderDoneCb_, this, _1, _2));
                action_code_ = coordinator::ManipTaskGoal::WAIT_FIND_TABLE_SURFACE;
                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                ROS_INFO("waiting on perception");
                break;
            case coordinator::ManipTaskGoal::WAIT_FIND_TABLE_SURFACE:
                if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FOUND) {
                    ROS_INFO("surface-finder success");
                    surface_height_ = pickup_pose_.pose.position.z; // table-top height, as found by object_finder
                    found_surface_height_ = true;
                    ROS_INFO("found table ht = %f", surface_height_);
                    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                    return; //done w/ callback
                } else if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FINDER_BUSY) {
                    //ROS_INFO("waiting on perception"); //do nothing
                } else {
                    ROS_WARN("object-finder failure; aborting");
                    action_code_ = coordinator::ManipTaskGoal::ABORT;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
                    found_surface_height_ = false;
                }

                break;

            case coordinator::ManipTaskGoal::GET_PICKUP_POSE:
                ROS_INFO("establishing pick-up pose");
                if (perception_source_ == coordinator::ManipTaskGoal::BLIND_MANIP) {
                    ROS_INFO("blind manipulation; using provided pick-up pose");
                    pickup_pose_ = goal->pickup_frame;
                    result_.object_pose = pickup_pose_;
                    //done with perception, but do fake waiting anyway
                    //declare victor on finding object
                    found_object_code_ = object_finder::objectFinderResult::OBJECT_FOUND;
                    action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_FINDER;
                    status_code_ = coordinator::ManipTaskFeedback::PERCEPTION_BUSY;
                } else if (perception_source_ == coordinator::ManipTaskGoal::PCL_VISION) {
                    ROS_INFO("invoking object finder");
                    found_object_code_ = object_finder::objectFinderResult::OBJECT_FINDER_BUSY;
                    ROS_INFO("instructing finder to locate object %d", vision_object_code_);
                    object_finder_goal_.object_id = vision_object_code_;
                    if (found_surface_height_) {
                        object_finder_goal_.known_surface_ht = true;
                        object_finder_goal_.surface_ht = surface_height_;
                        ROS_INFO("using surface ht = %f", surface_height_);
                    } else {
                        object_finder_goal_.known_surface_ht = false; //require find table height
                        //object_finder_goal_.surface_ht = 0.05; //not needed
                    }

                    ROS_INFO("sending object-finder goal: ");

                    object_finder_ac_.sendGoal(object_finder_goal_,
                            boost::bind(&TaskActionServer::objectFinderDoneCb_, this, _1, _2));

                    action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_FINDER;
                    ROS_INFO("waiting on perception");
                } else {
                    ROS_WARN("unrecognized perception mode; quitting");
                    action_code_ = coordinator::ManipTaskGoal::ABORT;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
                }

                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                break;

            case coordinator::ManipTaskGoal::WAIT_FOR_FINDER:
                if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FOUND) {
                    ROS_INFO("object-finder success");
                    //next step: use the pose to grab object:
                    /* if (goal_action_code_ == coordinator::ManipTaskGoal::MANIP_OBJECT) {
                        action_code_ = coordinator::ManipTaskGoal::GRAB_OBJECT;
                        status_code_ = coordinator::ManipTaskFeedback::DROPOFF_PLANNING_BUSY;
                    } else*/ {
                        working_on_task_ = false; // test--set to goal achieved
                        action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                        status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
                        result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
                        //object pose is in result message
                        as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                        return; //done w/ callback
                    }
                    //will later test for result code of object grabber, so initialize it to PENDING
                    //(next step in state machine)
                    object_grabber_return_code_ = object_grabber::object_grabberResult::PENDING;
                } else if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FINDER_BUSY) {
                    //ROS_INFO("waiting on perception"); //continue waiting
                } else {
                    ROS_WARN("object-finder failure; aborting");
                    action_code_ = coordinator::ManipTaskGoal::ABORT;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
                }
                break;

            case coordinator::ManipTaskGoal::GRAB_OBJECT:
                status_code_ = coordinator::ManipTaskFeedback::PICKUP_MOTION_BUSY;
                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                //ros::Duration(2.0).sleep();
                //if here, then presumably have a valid pose for object of interest; grab it! 
                //send object-grabber action code to grab specified object
                object_grabber_goal_.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT;//pickup_action_code_; //specify the object to be grabbed 
                object_grabber_goal_.object_frame = pickup_pose_; //and the object's current pose
                object_grabber_goal_.object_id = object_code_; // = goal->object_code;
                object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
                
                ROS_INFO("sending goal to grab object: ");
                object_grabber_ac_.sendGoal(object_grabber_goal_,
                        boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

                action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_GRAB_OBJECT;
                status_code_ = coordinator::ManipTaskFeedback::PICKUP_MOTION_BUSY;
                //will inspect this status to see if object grasp is eventually successful
                object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

                break;

            case coordinator::ManipTaskGoal::WAIT_FOR_GRAB_OBJECT:
                if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_ACQUIRED) { //success!
                    ROS_INFO("acquired object");


                    /*if (goal_action_code_ == coordinator::ManipTaskGoal::MANIP_OBJECT) {
                        //for manip command, have more steps to complete:
                        action_code_ = coordinator::ManipTaskGoal::DROPOFF_OBJECT;
                        status_code_ = coordinator::ManipTaskFeedback::PICKUP_SUCCESSFUL;
                    } else*/ { //if just a grab command, we are now done
                        working_on_task_ = false; // test--set to goal achieved
                        action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                        status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
                        result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
                        //object pose is in result message
                        as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                        return; //done w/ callback
                    }


                } else if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY) {
                    // do nothing--just wait patiently
                    //ROS_INFO("waiting for object grab");
                } else {
                    ROS_WARN("trouble with acquiring object");
                    action_code_ = coordinator::ManipTaskGoal::ABORT;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PICKUP;
                }
                break;

            case coordinator::ManipTaskGoal::CART_MOVE_TO_GRIPPER_POSE:
                status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                object_grabber_goal_.action_code = object_grabber::object_grabberGoal::CART_MOVE_CURRENT_TO_CART_GOAL;//code for cart move
                object_grabber_goal_.object_frame = goal->gripper_goal_frame; //get the move destination
                //object_grabber_goal_.object_id = object_code_; // = goal->object_code;
                //object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
                
                ROS_INFO("sending goal to move gripper along Cartesian path to specified destination: ");
                object_grabber_ac_.sendGoal(object_grabber_goal_,
                        boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

                action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;
                status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
                //will inspect this status to see if object grasp is eventually successful
                object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

                break;

             case coordinator::ManipTaskGoal::STRADDLE_OBJECT:
                status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                //ros::Duration(2.0).sleep();
                //if here, then presumably have a valid pose for object of interest; grab it! 
                //send object-grabber action code to grab specified object
                object_grabber_goal_.action_code = object_grabber::object_grabberGoal::STRADDLE_OBJECT;//pickup_action_code_; //specify the object to be grabbed 
                object_grabber_goal_.object_frame = pickup_pose_; //and the object's current pose
                object_grabber_goal_.object_id = object_code_; // = goal->object_code;
                object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
                
                ROS_INFO("sending goal to straddle object: ");
                object_grabber_ac_.sendGoal(object_grabber_goal_,
                        boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

                action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;
                status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
                //will inspect this status to see if object grasp is eventually successful
                object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

                break;
                
            case coordinator::ManipTaskGoal::DROPOFF_OBJECT:
                status_code_ = coordinator::ManipTaskFeedback::DROPOFF_MOTION_BUSY; //coordinator::ManipTaskResult::MANIP_SUCCESS; //code 0
                ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
                object_grabber_goal_.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT;//dropoff_action_code_; //specify the object to be grabbed 
                object_grabber_goal_.object_id = object_code_; // = goal->object_code;
                dropoff_pose_= goal->dropoff_frame;
                object_grabber_goal_.object_frame = dropoff_pose_; //and the object's current pose
                object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above

                        
                ROS_INFO("sending goal to drop off object: ");
                object_grabber_ac_.sendGoal(object_grabber_goal_,
                        boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

                action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_DROPOFF_OBJECT;
                //will inspect this status to see if object grasp is eventually successful
                object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

                break;


            case coordinator::ManipTaskGoal::WAIT_FOR_DROPOFF_OBJECT:
                //ROS_INFO("object_grabber_return_code_ = %d",object_grabber_return_code_);
                if (object_grabber_return_code_ == object_grabber::object_grabberResult::SUCCESS) { //success!
                    ROS_INFO("switch/case happiness!  dropped off object; manip complete");
                    working_on_task_ = false; // test--set to goal achieved
                    action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                    status_code_ = coordinator::ManipTaskFeedback::COMPLETED_DROPOFF;

                    result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
                    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                    return; //done w/ callback
                } else if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY) {
                    // do nothing--just wait patiently
                    //ROS_INFO("waiting for object dropoff");
                } else {
                    ROS_WARN("trouble with acquiring object");
                    action_code_ = coordinator::ManipTaskGoal::ABORT;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_DROPOFF;
                }
                break;

            case coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE:
                status_code_ = coordinator::ManipTaskFeedback::PREPOSE_MOVE_BUSY;
                object_grabber_goal_.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE; //specify the object to be grabbed 
                ROS_INFO("sending goal to move to pre-pose: ");
                object_grabber_ac_.sendGoal(object_grabber_goal_,
                        boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

                action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;
                //will inspect this status to see if object grasp is eventually successful
                object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;
                break;
            case coordinator::ManipTaskGoal::WAIT_FOR_MOVE:
                if (object_grabber_return_code_ == object_grabber::object_grabberResult::SUCCESS) { //success!
                    ROS_INFO("completed move");
                    working_on_task_ = false; // test--set to goal achieved
                    action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                    status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
                    result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
                    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                    return; //done w/ callback
                }
                else if(object_grabber_return_code_ == object_grabber::object_grabberResult::FAILED_CANNOT_REACH) {
                    ROS_WARN("unreachable");
                     working_on_task_ = false; // test--set to goal achieved
                    action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                    status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
                    result_.manip_return_code = coordinator::ManipTaskResult::FAILED_MOVE;
                    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
                    return; //done w/ callback            
                }
                else {
                    ROS_INFO("object_grabber_return_code_ = %d",object_grabber_return_code_);
                    ros::Duration(0.5).sleep();
                }

                break;

            case coordinator::ManipTaskGoal::ABORT:
                ROS_WARN("aborting goal...");
                //retain reason for failure to report back to client
                //result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
                action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
                status_code_ = coordinator::ManipTaskFeedback::ABORTED;
                working_on_task_ = false;
                as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
                return; // done with callback

            default:
                ROS_WARN("executeCB: error--case not recognized");
                working_on_task_ = false;
                break;
        }

    }
    ROS_INFO("executeCB: I should not be here...");
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
    as_.setAborted(result_); // return the "result" message to client, along with "success" status
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

