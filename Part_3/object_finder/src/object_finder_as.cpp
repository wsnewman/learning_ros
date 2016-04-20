// example_action_server: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<pcl_utils/pcl_utils.h>
#include<object_finder/objectFinderAction.h>


class ObjectFinder {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;
    
    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    PclUtils pclUtils_;
    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    bool find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose);

public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
};



ObjectFinder::ObjectFinder() :
   object_finder_as_(nh_, "objectFinderActionServer", boost::bind(&ObjectFinder::executeCB, this, _1),false),pclUtils_(&nh_) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation

    object_finder_as_.start(); //start the server running
}

//specialized function: DUMMY...JUST RETURN A HARD-CODED POSE; FIX THIS
bool ObjectFinder::find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose) {
    bool found_object=true;
    object_pose.header.frame_id = "torso";
    object_pose.pose.position.x = 0.680;
    object_pose.pose.position.y = -0.205;
    object_pose.pose.position.z = surface_height;
    object_pose.pose.orientation.x = 0;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0;
    object_pose.pose.orientation.w = 1;   
    return found_object;
    
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height=goal->surface_ht;
    }
    bool found_object=false;
    switch(object_id) {
        case object_finder::objectFinderGoal::COKE_CAN_UPRIGHT: 
              //specialized function to find an upright Coke can on a horizontal surface of known height:
               found_object = find_upright_coke_can(surface_height,object_pose); //special case for Coke can;
               if (found_object) {
                   ROS_INFO("found upright Coke can!");
                   result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                   result_.object_pose = object_pose;
                   object_finder_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not find requested object");
                   object_finder_as_.setAborted(result_);
               }
               break;
        default:
             ROS_WARN("this object ID is not implemented");
             result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED; 
             object_finder_as_.setAborted(result_);
            }
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

