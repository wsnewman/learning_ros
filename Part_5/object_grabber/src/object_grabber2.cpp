// object_grabber2: wsn, Dec, 2016
// add addl behaviors: STRADDLE_OBJECT and PLAN_CART_MOVE_CURRENT_TO_CART_GOAL
//define ObjectGrabber class; contains functionality to serve manipulation goals
// action server name is: object_grabber_action_service
// this layer accepts commands w/rt objects;
// it converts these to corresponding gripper poses, and uses a cartesian-move action server to compute
// and execute plans
// The cartesian-move action server is specific to a particular robot--but can expose a generic name for interactions
// Should be able to use generic frames: system_ref_frame and generic_gripper_frame
// Make sure there is a tf publisher for these for any robot that is launched

#include <object_grabber/object_grabber2.h>
using namespace std; 

ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle) : nh_(*nodehandle),
object_grabber_as_(nh_, "object_grabber_action_service", boost::bind(&ObjectGrabber::executeCB, this, _1), false),
 cart_move_action_client_("cartMoveActionServer", true) {
    ROS_INFO("in constructor of ObjectGrabber");
    //find out what type of gripper we have:
    if (!get_gripper_id()) {
        ROS_WARN("no gripper ID; quitting");
        exit(0);
  
    }


    manip_properties_client_ = nh_.serviceClient<object_manipulation_properties::objectManipulationQuery>("object_manip_query_svc");
    //attempt to connect to the service:
    ROS_INFO("waiting on manipulation-properties service: ");
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::TEST_PING;
    double t_test = 0;
    while (!manip_properties_client_.call(manip_properties_srv_)) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        t_test += 0.5;
        if (t_test > 2.0) ROS_WARN("can't connect to service object_manip_query_svc; is it running?");
    }
    
    gripper_client_ = nh_.serviceClient<generic_gripper_services::genericGripperInterface>("generic_gripper_svc");
    //attempt to connect to the service:
    ROS_INFO("waiting on gripper interface service: ");
    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::TEST_PING;
    t_test = 0;
    while (!gripper_client_.call(gripper_srv_)) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        t_test += 0.5;
        if (t_test > 2.0) ROS_WARN("can't connect to service generic_gripper_svc; is it running?");
    }    
    
    // attempt to connect to the server:
    /*
    ROS_INFO("waiting for cartesian-move action server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
     */
    object_grabber_as_.start(); //start the server running
}

bool ObjectGrabber::get_gripper_id() {
    if (nh_.getParam("gripper_ID", gripper_id_)) {
        ROS_INFO("found gripper ID %d on parameter server", gripper_id_);
        if (gripper_id_ == GripperIdCodes::RETHINK_ELECTRIC_GRIPPER_RT) {
            ROS_INFO("Baxter electric gripper");
        }
        return true;
    } else {
        ROS_WARN("could not find gripper ID on parameter server");
        return false;
    }
}

void ObjectGrabber::cartMoveDoneCb_(const actionlib::SimpleClientGoalState& state,
        const cartesian_planner::cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_ = *result;
}

//simplified use of manip_properties query: use the 0'th grasp option; intentionally construct
// the manip_properties service such that the 0'th option is the preferred/default option
// TODO: construct means to test viability of default option, and test/choose alternatives, if necessary
bool ObjectGrabber::get_default_grab_poses(int object_id,geometry_msgs::PoseStamped object_pose_stamped) {
    //fill in 3 necessary poses: approach, grasp, depart_w_object
    //find out what the default grasp strategy is for this gripper/object combination:
    manip_properties_srv_.request.gripper_ID = gripper_id_; //this is known from parameter server
    manip_properties_srv_.request.object_ID = object_id;
    manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_grasp_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d grasp options for this gripper/object combo; choosing 1st option (default)",n_grasp_strategy_options);
    if (n_grasp_strategy_options<1) return false;
    int grasp_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen grasp strategy is code %d",grasp_option);
    //use this grasp strategy for finding corresponding grasp pose

    
    manip_properties_srv_.request.grasp_option = grasp_option; //default option for grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;
    manip_properties_client_.call(manip_properties_srv_);
    int n_grasp_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_grasp_pose_options<1) {
                    ROS_WARN("no pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);;
                    return false;
                }
    
    grasp_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];
    //ROS_INFO("default grasped pose of object w/rt gripper: ");
    //xformUtils.printPose(grasp_object_pose_wrt_gripper_);
    
    //--------------now get the approach pose; first, find the default approach strategy:
     manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_approach_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d approach options for this gripper/object combo; choosing 1st option (default)",n_approach_strategy_options);
    if (n_approach_strategy_options<1) return false;
    int approach_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen approach strategy is code %d",approach_option);
    
    //use this grasp strategy for finding corresponding grasp pose
    manip_properties_srv_.request.grasp_option = approach_option; //default option for grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
     manip_properties_client_.call(manip_properties_srv_);
    int n_approach_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_approach_pose_options<1) {
                    ROS_WARN("no approach pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   
    approach_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; //using the 0'th, i.e. default option
    //ROS_INFO("default approach pose, expressed as pose of object w/rt gripper at approach: ");
    //xformUtils.printPose(approach_object_pose_wrt_gripper_);   

    //now get the depart (w/ grasped object) pose:  
    manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_depart_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d depart options for this gripper/object combo; choosing 1st option (default)",n_depart_strategy_options);
    if (n_depart_strategy_options<1) return false;
    int depart_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen depart strategy is code %d",depart_option);
    
    //use this grasp strategy for finding corresponding grasp pose
    manip_properties_srv_.request.grasp_option = depart_option; //default option for grasp strategy    
    
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
     manip_properties_client_.call(manip_properties_srv_);
    int n_depart_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_depart_pose_options<1) {
                    ROS_WARN("no depart pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   
    depart_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; //using the 0'th, i.e. default option
    //this is expressed somewhat strangely.  Often, this pose will be identical to approach_object_pose_wrt_gripper_
    //expresses original (ungrasped) pose of object from viewpoint of gripper when gripper is at the depart pose,
    // though this depart pose presumably would be holding the object of interest
    //ROS_INFO("default depart pose, expressed as original pose of object w/rt gripper at depart: ");
    //xformUtils.printPose(depart_object_pose_wrt_gripper_);   
    
    //use these relative values to compute gripper poses w/rt system ref frame--using the object's pose w/rt
    // it's named frame_id
    //object_pose_stamped
    //logic: grasp_object_pose_wrt_gripper_: can say object pose w/rt generic_gripper_frame;
    // also know object pose w/rt some named frame, e.g. system_ref_frame
    // use these to solve for generic_gripper_frame w/rt some named frame (e.g. object's frame_id)
    // given A_obj/gripper and A_obj/sys --> A_gripper/sys
    //stf of object frame w/rt its specified frame_id (from object_pose_stamped)
    ROS_INFO("computing grasp stf: ");
    tf::StampedTransform object_stf = 
            xformUtils.convert_poseStamped_to_stampedTransform(object_pose_stamped, "object_frame");
    geometry_msgs::PoseStamped object_wrt_gripper_ps;
    object_wrt_gripper_ps.pose = grasp_object_pose_wrt_gripper_;
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    tf::StampedTransform object_wrt_gripper_stf = 
            xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    tf::StampedTransform gripper_stf;
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    grasp_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
    //ROS_INFO("computed gripper pose at grasp location: ");
    //xformUtils.printStampedPose(grasp_pose_);
    
    //do same for approach and depart poses:
    //approach_object_pose_wrt_gripper_
    ROS_INFO("computing approach stf: ");
    object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_; //transform approach pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);    
    // ROS_INFO("computed gripper pose at approach location: ");
    //xformUtils.printStampedPose(approach_pose_);   
    
    //finally, repeat for depart pose:
    //ROS_INFO("computing depart stf: ");
    object_wrt_gripper_ps.pose = depart_object_pose_wrt_gripper_; //transform depart pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);      
    // ROS_INFO("computed gripper pose at depart location: ");
    //xformUtils.printStampedPose(depart_pose_);      
    
    return true;
}

//compute gripper poses for default drop-off strategy--which will be strategy option 0
// this will be identical to the corresponding object/gripper combo grasp strategy,
// but poses will be computed based on the desired dropoff coordinates
// TODO: construct means to test viability of the default option, and test/choose alternatives, if necessary
// e.g., default option might be unreachable, or may result in collisions w/ self or environment
bool ObjectGrabber::get_default_dropoff_poses(int object_id,geometry_msgs::PoseStamped object_dropoff_pose_stamped) {
    //fill in 3 necessary poses: approach_w_object, release, depart_wo_object
    //find out what the default grasp strategy is for this gripper/object combination:
    manip_properties_srv_.request.gripper_ID = gripper_id_; //this is known from parameter server
    manip_properties_srv_.request.object_ID = object_id;
    manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_grasp_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d grasp options for this gripper/object combo; choosing 1st option (default)",n_grasp_strategy_options);
    if (n_grasp_strategy_options<1) return false;
    int grasp_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen grasp strategy is code %d",grasp_option);
    //use this grasp strategy for finding corresponding grasp pose

    
    manip_properties_srv_.request.grasp_option = grasp_option; //default option for grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;
    manip_properties_client_.call(manip_properties_srv_);
    int n_grasp_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_grasp_pose_options<1) {
                    ROS_WARN("no pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);;
                    return false;
                }
    
    grasp_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];
    ROS_INFO("default grasped pose of object w/rt gripper: ");
    xformUtils.printPose(grasp_object_pose_wrt_gripper_);
    
    //--------------now get the approach pose; first, find the default approach strategy:
     manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_approach_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d approach options for this gripper/object combo; choosing 1st option (default)",n_approach_strategy_options);
    if (n_approach_strategy_options<1) return false;
    int approach_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen approach strategy is code %d",approach_option);
    
    //use this grasp strategy for finding corresponding grasp pose
    manip_properties_srv_.request.grasp_option = approach_option; //default option for grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
     manip_properties_client_.call(manip_properties_srv_);
    int n_approach_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_approach_pose_options<1) {
                    ROS_WARN("no approach pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   
    approach_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; //using the 0'th, i.e. default option
    ROS_INFO("default approach pose, expressed as pose of object w/rt gripper at approach: ");
    xformUtils.printPose(approach_object_pose_wrt_gripper_);   

    //now get the depart (w/ grasped object) pose:  
    manip_properties_srv_.request.query_code =
            object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    int n_depart_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d depart options for this gripper/object combo; choosing 1st option (default)",n_depart_strategy_options);
    if (n_depart_strategy_options<1) return false;
    int depart_option = manip_properties_srv_.response.grasp_strategy_options[0];
    ROS_INFO("chosen depart strategy is code %d",depart_option);
    
    //use this grasp strategy for finding corresponding grasp pose
    manip_properties_srv_.request.grasp_option = depart_option; //default option for grasp strategy    
    
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
     manip_properties_client_.call(manip_properties_srv_);
    int n_depart_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_depart_pose_options<1) {
                    ROS_WARN("no depart pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   
    depart_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; //using the 0'th, i.e. default option
    //this is expressed somewhat strangely.  Often, this pose will be identical to approach_object_pose_wrt_gripper_
    //expresses original (ungrasped) pose of object from viewpoint of gripper when gripper is at the depart pose,
    // though this depart pose presumably would be holding the object of interest
    ROS_INFO("default depart pose, expressed as original pose of object w/rt gripper at depart: ");
    xformUtils.printPose(depart_object_pose_wrt_gripper_);   
    
    //use these relative values to compute gripper poses w/rt system ref frame--using the object's pose w/rt
    // it's named frame_id
    //object_dropoff_pose_stamped
    //logic: grasp_object_pose_wrt_gripper_: can say object pose w/rt generic_gripper_frame;
    // also know object pose w/rt some named frame, e.g. system_ref_frame
    // use these to solve for generic_gripper_frame w/rt some named frame (e.g. object's frame_id)
    // given A_obj/gripper and A_obj/sys --> A_gripper/sys
    //stf of object frame w/rt its specified frame_id (from object_dropoff_pose_stamped)
    ROS_INFO("computing dropoff stf: ");
    tf::StampedTransform object_stf = 
            xformUtils.convert_poseStamped_to_stampedTransform(object_dropoff_pose_stamped, "object_frame");
    geometry_msgs::PoseStamped object_wrt_gripper_ps;
    object_wrt_gripper_ps.pose = grasp_object_pose_wrt_gripper_;
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    tf::StampedTransform object_wrt_gripper_stf = 
            xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    ROS_INFO("object w/rt gripper stf: ");
    xformUtils.printStampedTf(object_wrt_gripper_stf);
    tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    ROS_INFO("gripper w/rt object stf: ");
    xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    tf::StampedTransform gripper_dropoff_stf;
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_dropoff_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    dropoff_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_dropoff_stf);
    ROS_INFO("computed gripper pose at dropoff location: ");
    xformUtils.printStampedPose(dropoff_pose_);
    
    //do same for approach and depart poses:
    //approach_object_pose_wrt_gripper_
    ROS_INFO("computing approach stf: ");
    object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_; //transform approach pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    ROS_INFO("object w/rt gripper stf: ");
    xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    ROS_INFO("gripper w/rt object stf: ");
    xformUtils.printStampedTf(gripper_wrt_object_stf);
    tf::StampedTransform gripper_approach_stf;
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_approach_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_approach_stf);    
     ROS_INFO("computed gripper pose at approach location: ");
    xformUtils.printStampedPose(approach_pose_);   
    
    //finally, repeat for depart pose:
    ROS_INFO("computing depart stf: ");
    object_wrt_gripper_ps.pose = depart_object_pose_wrt_gripper_; //transform depart pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    ROS_INFO("object w/rt gripper stf: ");
    xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    ROS_INFO("gripper w/rt object stf: ");
    xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    tf::StampedTransform gripper_depart_stf;
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_depart_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_depart_stf);      
     ROS_INFO("computed gripper pose at depart location: ");
    xformUtils.printStampedPose(depart_pose_);      
    
    return true;
}

//grab-object function w/ 2 args is default strategy for approach, grasp depart:
int  ObjectGrabber::grab_object(int object_id,geometry_msgs::PoseStamped object_pose_stamped){
    //given gripper_ID, object_ID and object poseStamped,
    // and assuming default approach, grasp and depart strategies for this object/gripper combo,
    // compute the corresponding required gripper-frame poses w/rt a named frame_id
    // (which will be same frame_id as specified in object poseStamped)
    int rtn_val;
    bool success;
    if(!get_default_grab_poses(object_id,object_pose_stamped)) {
        ROS_WARN("no valid grasp strategy; giving up");
        return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
    }
    //invoke the sequence of moves to perform approach, grasp, depart:
    ROS_WARN("prepare gripper state to anticipate grasp...");
    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
    gripper_client_.call(gripper_srv_); 
    success = gripper_srv_.response.success;
    if (success) { ROS_INFO("gripper responded w/ success"); }
    else {ROS_WARN("responded with failure"); }
    
    ROS_WARN("object-grabber as planning joint-space move to approach pose");
    //xformUtils.printPose(approach_pose_);

    rtn_val=arm_motion_commander_.plan_jspace_path_current_to_cart_gripper_pose(approach_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
        
    //send command to execute planned motion
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    //ros::Duration(2.0).sleep();
    
    ROS_INFO("planning motion of gripper to grasp pose at: ");
    xformUtils.printPose(grasp_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(grasp_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();   
    ROS_WARN("poised to grasp object; invoke gripper grasp action here ...");

    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::GRASP;
    gripper_client_.call(gripper_srv_); 
    success = gripper_srv_.response.success;
    ros::Duration(1.0).sleep(); //add some extra time to stabilize grasp...tune this
    if (success) { ROS_INFO("gripper responded w/ success"); }
    else {ROS_WARN("responded with failure"); }    
 
    ROS_INFO("planning motion of gripper to depart pose at: ");
    xformUtils.printPose(depart_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(depart_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    ROS_INFO("performing motion");
    rtn_val=arm_motion_commander_.execute_planned_path();
 
    
    return rtn_val; 
}

//straddle object: this is just the first part of grab-object; go to grasp pose and stay there
int  ObjectGrabber::straddle_object(int object_id,geometry_msgs::PoseStamped object_pose_stamped){
    //given gripper_ID, object_ID and object poseStamped,
    // and assuming default approach, grasp and depart strategies for this object/gripper combo,
    // compute the corresponding required gripper-frame poses w/rt a named frame_id
    // (which will be same frame_id as specified in object poseStamped)
    int rtn_val;
    bool success;
    if(!get_default_grab_poses(object_id,object_pose_stamped)) {
        ROS_WARN("no valid grasp strategy; giving up");
        return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
    }
    //invoke the sequence of moves to perform approach, grasp, depart:
    ROS_WARN("prepare gripper state to anticipate grasp...");
    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
    gripper_client_.call(gripper_srv_); 
    success = gripper_srv_.response.success;
    if (success) { ROS_INFO("gripper responded w/ success"); }
    else {ROS_WARN("responded with failure"); }
    
    ROS_WARN("object-grabber action server is planning joint-space move to approach pose");
    //xformUtils.printPose(approach_pose_);

    rtn_val=arm_motion_commander_.plan_jspace_path_current_to_cart_gripper_pose(approach_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
        
    //send command to execute planned motion
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    //ros::Duration(2.0).sleep();
    
    ROS_INFO("planning motion of gripper to grasp pose at: ");
    xformUtils.printPose(grasp_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(grasp_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();   
    ROS_WARN("concluded motion; gripper should be in grasp pose");
 
    return rtn_val; 
}


//grab-object function w/ 2 args is default strategy for approach, grasp depart:
int  ObjectGrabber::dropoff_object(int object_id,geometry_msgs::PoseStamped desired_object_pose_stamped){
    //given gripper_ID, object_ID and object poseStamped,
    // and assuming default approach, grasp and depart strategies for this object/gripper combo,
    // compute the corresponding required gripper-frame poses w/rt a named frame_id
    // (which will be same frame_id as specified in object poseStamped)
    int rtn_val;
    bool success;

    if(!get_default_dropoff_poses(object_id,desired_object_pose_stamped)) {
        ROS_WARN("no valid grasp strategy; giving up");
        return object_grabber::object_grabberResult::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
    }
    //invoke the sequence of moves to perform approach, grasp, depart:
    ROS_INFO("planning move to approach pose");
    ROS_INFO("planning motion of gripper to approach pose at: ");
    xformUtils.printPose(approach_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(approach_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
        
    //send command to execute planned motion
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    //ros::Duration(2.0).sleep();
    
    ROS_INFO("planning motion of gripper to dropoff pose at: ");
    xformUtils.printPose(dropoff_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(dropoff_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    ROS_INFO("executing plan: ");
    rtn_val=arm_motion_commander_.execute_planned_path();   
    ROS_INFO("poised to release object;  invoke gripper release action here ...");
   
    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
    gripper_client_.call(gripper_srv_); 
    success = gripper_srv_.response.success;
    if (success) { ROS_INFO("gripper responded w/ success"); }
    else {ROS_WARN("responded with failure"); }    
    ros::Duration(1.0).sleep();
    
 
    ROS_INFO("planning motion of gripper to depart pose at: ");
    xformUtils.printPose(depart_pose_);
    rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(depart_pose_);
    if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) return rtn_val; //return error code
    ROS_INFO("performing motion");
    rtn_val=arm_motion_commander_.execute_planned_path();
 
    return rtn_val; 
}



//cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ObjectGrabber::cartMoveDoneCb_, this, _1, _2));

void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {
    int action_code = goal->action_code;
    ROS_INFO("got action code %d", action_code);
    int object_grabber_rtn_code, rtn_val;
    int object_id;
    int grasp_option; //code for one of N enumerated grasp-strategy options relevant to a given object/gripper combo
    //options for how to approach a part, depart while holding the part, 
    // approach to deposit a part, and gripper withdrawal strategy after releasing a part
    int approach_strategy, lift_object_strategy, dropoff_strategy, dropoff_withdraw_strategy;
    bool have_default_grasp_plan;
    switch (action_code) {
        case object_grabber::object_grabberGoal::TEST_CODE:
            ROS_INFO("got test ping");
            arm_motion_commander_.send_test_goal(); // send a test command
            grab_result_.return_code = object_grabber::object_grabberResult::SUCCESS;
            object_grabber_as_.setSucceeded(grab_result_);
            break;

            //need ability to get hand out of way of camera
        case object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE:
            ROS_INFO("planning move to waiting pose");
            rtn_val = arm_motion_commander_.plan_move_to_waiting_pose(); //this should always be successful
            ROS_INFO("commanding plan execution");
            rtn_val = arm_motion_commander_.execute_planned_path();
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            //cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE;
            //cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ObjectGrabber::cartMoveDoneCb_, this, _1, _2));
            //ROS_INFO("return code: %d", cart_result_.return_code);
            break;

        case object_grabber::object_grabberGoal::GRAB_OBJECT:
            ROS_INFO("GRAB_OBJECT: ");
            object_id = goal->object_id;
            grasp_option = goal->grasp_option;
            object_pose_stamped_ = goal->object_frame;
            //get grasp-plan details for this case:
            if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
            {
                ROS_WARN("grasp strategy %d not implemented yet; using default strategy",grasp_option);
            }
            rtn_val = grab_object(object_id,object_pose_stamped_);
            ROS_INFO("grasp attempt concluded");
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
            
        case object_grabber::object_grabberGoal::STRADDLE_OBJECT:
            ROS_INFO("STRADDLE_OBJECT: ");
            object_id = goal->object_id;
            grasp_option = goal->grasp_option;
            object_pose_stamped_ = goal->object_frame;
            //get grasp-plan details for this case:
            if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
            {
                ROS_WARN("grasp strategy %d not implemented yet; using default strategy",grasp_option);
            }
            rtn_val = straddle_object(object_id,object_pose_stamped_);
            ROS_INFO("straddle attempt concluded");
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
            
        case object_grabber::object_grabberGoal::DROPOFF_OBJECT:
            ROS_INFO("DROPOFF_OBJECT: ");
            object_id = goal->object_id;
            grasp_option = goal->grasp_option;
            object_pose_stamped_ = goal->object_frame;
            //get grasp-plan details for this case:
            if (grasp_option != object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY)
            {
                ROS_WARN("grasp strategy %d not implemented yet; using default strategy",grasp_option);
            }
            rtn_val = dropoff_object(object_id,object_pose_stamped_);
            ROS_INFO("dropoff attempt concluded");
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
           
        //new case to command a Cartesian move to a specified Cartesian goal
        //get goal_pose_stamped_ as interpretation of goal->object_frame element
        case object_grabber::object_grabberGoal::CART_MOVE_CURRENT_TO_CART_GOAL:
            ROS_INFO("planning Cartesian move from current pose to goal pose");
            goal_pose_stamped_ = goal->object_frame;
            rtn_val=arm_motion_commander_.plan_path_current_to_goal_gripper_pose(goal_pose_stamped_);
            if (rtn_val != cartesian_planner::cart_moveResult::SUCCESS) { //return error code
               grab_result_.return_code = object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
               object_grabber_as_.setSucceeded(grab_result_);
            }
            else {
             //if here, plan is valid, so send command to execute planned motion
             ROS_INFO("executing plan: ");
             rtn_val=arm_motion_commander_.execute_planned_path();
             grab_result_.return_code = rtn_val;
             object_grabber_as_.setSucceeded(grab_result_);   
            }
            break;
            
        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }
}
