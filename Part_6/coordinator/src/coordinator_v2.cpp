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

geometry_msgs::PoseStamped g_perceived_object_pose;
int g_found_object_code;
int g_object_grabber_return_code;
int g_navigator_rtn_code;

const int FIND_BLOCK = 100;
const int GRAB_BLOCK = 101;
const int NAVIGATE = 102;
const int DO_NOTHING = 0;
const int ABORT = 1;
//const int BUSY = 2;
const int BUSY_FINDER = 200;
const int BUSY_GRABBER = 201;

bool g_action_trigger = false;
int g_action_code = DO_NOTHING;

ros::Publisher *g_pose_publisher; //useful for publishing perceived object pose marker

void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code = result->found_object_code;
    ROS_INFO("got object code response = %d; ", g_found_object_code);
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    } else if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
        g_perceived_object_pose = result->object_pose;
        ROS_INFO("got pose x,y,z = %f, %f, %f", g_perceived_object_pose.pose.position.x,
                g_perceived_object_pose.pose.position.y,
                g_perceived_object_pose.pose.position.z);

        ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f", g_perceived_object_pose.pose.orientation.x,
                g_perceived_object_pose.pose.orientation.y,
                g_perceived_object_pose.pose.orientation.z,
                g_perceived_object_pose.pose.orientation.w);
        g_pose_publisher->publish(g_perceived_object_pose);
    } else {
        ROS_WARN("object not found!");
    }
}

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ", result->return_code);
    g_object_grabber_return_code = result->return_code;
}

/* ADD NAVIGATOR HERE xxx
void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const navigator::navigatorResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code = result->return_code;
    ROS_INFO("got object code response = %d; ", g_navigator_rtn_code);
    if (g_navigator_rtn_code == navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    } else if (g_navigator_rtn_code == navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    } else {
        ROS_WARN("desired pose not reached!");
    }
}
 */

//external trigger:

void actionCB(const std_msgs::Int32& code_msg) {
    int action_code = code_msg.data;
    ROS_INFO("received action code: %d", action_code);
    g_action_trigger = true; //maybe redundant--could infer from action_code=0
    g_action_code = action_code;
}

bool svc_callback(coordinator::CoordinatorSrvRequest& request, coordinator::CoordinatorSrvResponse& response)
{
    g_action_trigger = true; //maybe redundant--could infer from action_code=0
    g_action_code = request.action_code; 
        ROS_INFO("service callback received action code %d",g_action_code);
    response.rtn_code = 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    

    ros::Subscriber action_code = nh.subscribe("/action_codes", 1, actionCB);

    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("objectFinderActionServer", true);
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("objectGrabberActionServer", true);

    //xxx FIX ME
    //actionlib::SimpleActionClient<navigator::navigatorAction> navigator_ac("navigatorActionServer", true);
    
      ros::ServiceServer service = nh.advertiseService("coordinator_svc", svc_callback);

    // attempt to connect to the object-finder server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    //use this to visualize computed object poses
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    g_pose_publisher = &pose_publisher;


    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 


    //do the same with the "navigator" action server
    // attempt to connect to the server:
    //xxx FIX ME--add navigator
    /*
    ROS_INFO("waiting for server: ");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to navigator action server"); // if here, then we connected to the server; 
     */
    //specifications for what we are seeking:
    object_finder::objectFinderGoal object_finder_goal;
    object_grabber::object_grabberGoal object_grabber_goal;

    // FIX ME xxx
    //navigator::navigatorGoal navigation_goal;

    bool finished_before_timeout;

    //ALL SET UP; WAITING FOR TRIGGER
    //wait for the action trigger:
    ROS_INFO("waiting for action code: rostopic pub action_codes std_msgs/Int32 100");
    while (ros::ok()) {
        ros::spinOnce();
        switch (g_action_code) {
            case DO_NOTHING:
                ros::Duration(0.5).sleep();
                break;

            case BUSY_FINDER: // also do nothing...but maybe comment on status? set a watchdog?
                if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FINDER_BUSY) {
                    ROS_INFO("object finder is busy processing...");
                    ros::Duration(0.5).sleep();
                    break;
                }
                if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) { //success!
                    ROS_INFO("switch/case happiness!  found object");
                    g_action_code = DO_NOTHING;
                    break;
                }
                //if here, bad case:
                ROS_WARN("did not find object");
                g_action_code = DO_NOTHING;
                break;

            case BUSY_GRABBER:
                if (g_object_grabber_return_code == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY) {
                    ROS_INFO("object grabber is busy executing...");
                    ros::Duration(0.5).sleep();
                    break;
                }
                if (g_object_grabber_return_code == object_grabber::object_grabberResult::OBJECT_ACQUIRED) { //success!
                    ROS_INFO("switch/case happiness!  acquired object");
                    g_action_code = DO_NOTHING;
                    break;
                }
                ROS_WARN("did not grab object");
                g_action_code = DO_NOTHING;
                break;


            case ABORT:
                ROS_INFO("aborting operations in progress: ");
                if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FINDER_BUSY) {
                    object_finder_ac.cancelGoal();
                    g_found_object_code == object_finder::objectFinderResult::OBJECT_FINDER_CANCELLED;
                    g_action_code = DO_NOTHING;
                }
                if (g_object_grabber_return_code == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY) {
                    object_grabber_ac.cancelGoal();
                    g_object_grabber_return_code = object_grabber::object_grabberResult::OBJECT_GRABBER_CANCELLED;
                    g_action_code = DO_NOTHING;
                }
                //do same for navigation
                break;


            case FIND_BLOCK:
                //set flag indicating waiting on response from object finder
                // this will get reset by object-finder callback
                // need to consider time-outs
                ROS_INFO("received find-block command code; starting work...");

                g_found_object_code = object_finder::objectFinderResult::OBJECT_FINDER_BUSY;
                //populate a goal message to find TOY_BLOCK
                object_finder_goal.object_id = object_finder::objectFinderGoal::TOY_BLOCK;
                object_finder_goal.known_surface_ht = false; //require find table height
                object_finder_goal.surface_ht = 0.05;

                object_finder_ac.sendGoal(object_finder_goal, &objectFinderDoneCb);
                g_action_code = BUSY_FINDER;
                break;

            case GRAB_BLOCK:
                ROS_INFO("received grab-block command code; starting work...");
                g_object_grabber_return_code = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;
                //populate a goal message to grab TOY_BLOCK
                object_grabber_goal.object_code = object_grabber::object_grabberGoal::TOY_BLOCK; //specify the object to be grabbed
                object_grabber_goal.object_frame = g_perceived_object_pose;

                object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
                g_action_code = BUSY_GRABBER;
                break;

            case NAVIGATE:
                ROS_INFO("not implemented yet");
                g_action_code = DO_NOTHING;

                break;
        }



        /*
        ROS_INFO("sending navigation goal: TABLE");
        navigation_goal.location_code = navigator::navigatorGoal::TABLE; //send robot to TABLE
        navigator_ac.sendGoal(navigation_goal, &navigatorDoneCb); // we could also name additional callback functions here, if desired
        finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        //SHOULD do error checking here before proceeding...
        if (g_navigator_rtn_code != navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
            ROS_WARN("COULD NOT REACH TABLE; QUITTING");
            return 1;
        }
         */

    }
    return 0;
}

