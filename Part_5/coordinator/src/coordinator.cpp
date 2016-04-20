// coordinator: 
// wsn, April, 2016
// illustrates use of object_finder, object_grasper and navigator action servers

//trigger this process with:
//rostopic pub Alexa_codes std_msgs/UInt32 100

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

geometry_msgs::PoseStamped g_perceived_object_pose;
int g_found_object_code;
int g_object_grabber_return_code;
int g_navigator_rtn_code;
bool g_get_coke_trigger=false;


const int ALEXA_GET_COKE_CODE= 100;

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
    }
    else {
        ROS_WARN("object not found!");
    }
}

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ",result->return_code);
    g_object_grabber_return_code = result->return_code;
}


void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const navigator::navigatorResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code=result->return_code;
    ROS_INFO("got object code response = %d; ",g_navigator_rtn_code);
    if (g_navigator_rtn_code==navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    }
    else if (g_navigator_rtn_code==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    }
    else {
        ROS_WARN("desired pose not reached!");
    }
}

//external trigger:
void alexaCB(const std_msgs::UInt32& code_msg) {
    int alexa_code = code_msg.data;
    ROS_INFO("received Alexa code: %d", alexa_code);
    if (alexa_code= ALEXA_GET_COKE_CODE) {
      g_get_coke_trigger = true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    ros::Subscriber alexa_code = nh.subscribe("/Alexa_codes", 1, alexaCB);
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("objectFinderActionServer", true);
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("objectGrabberActionServer", true);
    actionlib::SimpleActionClient<navigator::navigatorAction> navigator_ac("navigatorActionServer", true);

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
    //connect to the object_grabber server
    server_exists=false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 
    
    //do the same with the "navigator" action server
     // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to navigator action server"); // if here, then we connected to the server; 

    //specifications for what we are seeking:
    object_finder::objectFinderGoal object_finder_goal;   
    object_grabber::object_grabberGoal object_grabber_goal;
    navigator::navigatorGoal navigation_goal;
    
    bool finished_before_timeout;
    
    //ALL SET UP; WAITING FOR TRIGGER
    //wait for the Alexa trigger:
    ROS_INFO("waiting for Alexa code: rostopic pub Alexa_codes std_msgs/UInt32 100");
    while (!g_get_coke_trigger) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();    
    }

    //  IF HERE, START THE FETCH BEHAVIOR!!
    
    ROS_INFO("sending navigation goal: TABLE");
    navigation_goal.location_code=navigator::navigatorGoal::TABLE; //send robot to TABLE
        navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb); // we could also name additional callback functions here, if desired
        finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
      //SHOULD do error checking here before proceeding...
        if (g_navigator_rtn_code!= navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
            ROS_WARN("COULD NOT REACH TABLE; QUITTING");
            return 1;
        }
                
                
    //assume we have reached the table; look for the Coke can:
    object_finder_goal.object_id=object_finder::objectFinderGoal::COKE_CAN_UPRIGHT; //specify object of interest
    object_finder_goal.known_surface_ht=true; //we'll say we know the table height
    object_finder_goal.surface_ht = 0.05;  // and specify the height, relative to torso; TUNE THIS
    //try to find the object:
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        //decide how long we will wait
        finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1; // halt with failure
        }
     //SHOULD examine the return code,   
        if (g_found_object_code!= object_finder::objectFinderResult::OBJECT_FOUND) {
            ROS_WARN("could not find object; quitting!");
            return 1;
        }
     //if here, then presumably have a valid pose for object of interest; grab it!       
        object_grabber_goal.object_code = object_grabber::object_grabberGoal::COKE_CAN; //specify the object to be grabbed 
    object_grabber_goal.object_frame = g_perceived_object_pose;
    ROS_INFO("sending goal to grab object: ");
        object_grabber_ac.sendGoal(object_grabber_goal,&objectGrabberDoneCb); 
        //decide how long to wait...
        finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(40.0));

        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result; quitting ");
            return 1;
        }
        if (g_object_grabber_return_code!= object_grabber::object_grabberResult::OBJECT_ACQUIRED) {
            ROS_WARN("failed to grab object; giving up!");
            return 1;
        }
        
        //if here, belief is that we are holding the Coke; return home            
    ROS_INFO("sending navigation goal: HOME");
    navigation_goal.location_code=navigator::navigatorGoal::HOME; //send robot to TABLE
        navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb); // we could also name additional callback functions here, if desired
        finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
      //SHOULD do error checking here before proceeding...
        if (g_navigator_rtn_code!= navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
            ROS_WARN("COULD NOT REACH TABLE; QUITTING");
            return 1;
        }
  ROS_INFO("My work here is done!"); //more generally, keep this behavior alive      
        
    return 0;
}

