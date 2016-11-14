// object_grabber(v3): wsn, Nov, 2016
//define ObjectGrabber class; contains functionality to serve manipulation goals
// action server name is: object_grabber_action_service
// this layer accepts commands w/rt objects;
// it converts these to corresponding gripper poses, and uses a cartesian-move action server to compute
// and execute plans
// The cartesian-move action server is specific to a particular robot--but can expose a generic name for interactions
// Should be able to use generic frames: system_ref_frame and generic_gripper_frame
// Make sure there is a tf publisher for these for any robot that is launched

#include <object_grabber/object_grabber3.h>
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
    ROS_INFO("object w/rt gripper stf: ");
    xformUtils.printStampedTf(object_wrt_gripper_stf);
    tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    ROS_INFO("gripper w/rt object stf: ");
    xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    tf::StampedTransform gripper_stf;
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    grasp_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
    ROS_INFO("computed gripper pose at grasp location: ");
    xformUtils.printStampedPose(grasp_pose_);
    
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
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);    
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
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);      
     ROS_INFO("computed gripper pose at depart location: ");
    xformUtils.printStampedPose(depart_pose_);      
    
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
        return object_grabber::object_grabber3Result::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
    }
    //invoke the sequence of moves to perform approach, grasp, depart:
    ROS_WARN("prepare gripper state to anticipate grasp...");
    gripper_srv_.request.cmd_code = generic_gripper_services::genericGripperInterfaceRequest::RELEASE;
    gripper_client_.call(gripper_srv_); 
    success = gripper_srv_.response.success;
    if (success) { ROS_INFO("gripper responded w/ success"); }
    else {ROS_WARN("responded with failure"); }
    
    ROS_INFO("planning joint-space move to approach pose");
    
    ROS_INFO("planning motion of gripper to approach pose at: ");
    xformUtils.printPose(approach_pose_);
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
        return object_grabber::object_grabber3Result::NO_KNOWN_GRASP_OPTIONS_THIS_GRIPPER_AND_OBJECT;
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

void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabber3Action>::GoalConstPtr& goal) {
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
        case object_grabber::object_grabber3Goal::TEST_CODE:
            ROS_INFO("got test ping");
            arm_motion_commander_.send_test_goal(); // send a test command
            grab_result_.return_code = object_grabber::object_grabber3Result::SUCCESS;
            object_grabber_as_.setSucceeded(grab_result_);
            break;

            //need ability to get hand out of way of camera
        case object_grabber::object_grabber3Goal::MOVE_TO_WAITING_POSE:
            ROS_INFO("planning move to waiting pose");
            rtn_val = arm_motion_commander_.plan_move_to_pre_pose(); //this should always be successful
            ROS_INFO("commanding plan execution");
            rtn_val = arm_motion_commander_.execute_planned_path();
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            //cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE;
            //cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ObjectGrabber::cartMoveDoneCb_, this, _1, _2));
            //ROS_INFO("return code: %d", cart_result_.return_code);
            break;

        case object_grabber::object_grabber3Goal::GRAB_OBJECT:
            ROS_INFO("GRAB_OBJECT: ");
            object_id = goal->object_id;
            grasp_option = goal->grasp_option;
            object_pose_stamped_ = goal->object_frame;
            //get grasp-plan details for this case:
            if (grasp_option != object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY)
            {
                ROS_WARN("grasp strategy %d not implemented yet; using default strategy",grasp_option);
            }
            rtn_val = grab_object(object_id,object_pose_stamped_);
            ROS_INFO("grasp attempt concluded");
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabber3Goal::DROPOFF_OBJECT:
            ROS_INFO("DROPOFF_OBJECT: ");
            object_id = goal->object_id;
            grasp_option = goal->grasp_option;
            object_pose_stamped_ = goal->object_frame;
            //get grasp-plan details for this case:
            if (grasp_option != object_grabber::object_grabber3Goal::DEFAULT_GRASP_STRATEGY)
            {
                ROS_WARN("grasp strategy %d not implemented yet; using default strategy",grasp_option);
            }
            rtn_val = dropoff_object(object_id,object_pose_stamped_);
            ROS_INFO("dropoff attempt concluded");
            grab_result_.return_code = rtn_val;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
            
        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabber3Result::ACTION_CODE_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }
}
// function to grasp an object (e.g. toy_block) from above;
// provide object pose assuming pose origin is on top surface of object 
// assume object frame z-axis is "up" (normal to top face, pointing outwards)
// and object-frame x-axis is along the object's major axis 
// grasp pose for this fnc will have fingers below top surface of block,  e.g. delta-z = 0.03 should work for toy_block,
// and will assume fingers straddle the object along the minor axis
// start from safe distance above (approach pose); descend to grasp pose (carefully); depart back to approach pose

// grasp_z_offset: starting from gripper tip flush with top surface of object, gripper should descend this much before grasping object
//  this value depends on the specific object and should be encoded with object-type description

//note--block_pose_wrt_torso is already assured to be w/rt torso, due to vetting by executeCB()

//fnc to make a Cartesian move to specified flange pose:

/*
int ObjectGrabber::move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    std::vector<Vectorq7x1> q_solns;
    //plan move from current pose to approach pose:
    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);

    //is plan successful?
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to specified approach pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_POSE_CARTESIAN_MOVE;
    }

    //if here, plan is good, so send command to execute planned motion
    ROS_INFO("sending command to execute planned path to approach pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS;
}

int ObjectGrabber::jspace_move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    std::vector<Vectorq7x1> q_solns;
    //plan move from current pose to approach pose:
    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.rt_arm_plan_jspace_path_current_to_flange_pose(des_flange_pose_wrt_torso);
    //is plan successful?
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to specified approach pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH_JSPACE_MOVE;
    }

    //if here, plan is good, so send command to execute planned motion
    ROS_INFO("sending command to execute planned path to approach pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS;
}

int ObjectGrabber::jspace_move_to_pre_pose(void) {
    //plan move from current pose to approach pose:
    int planner_rtn_code, execute_return_code;
    planner_rtn_code = armMotionCommander.plan_move_to_pre_pose();

    //is plan successful?
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("cannot move to pre-pose");
        return object_grabber::object_grabberResult::FAILED_CANNOT_MOVE_TO_PRE_POSE;
    }

    //if here, plan is good, so send command to execute planned motion
    ROS_INFO("sending command to execute planned path to pre-pose:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS;
}

//pure, careful, precise motion from current pose to precise goal...e.g. vertical approach

int ObjectGrabber::fine_move_flange_to(geometry_msgs::PoseStamped des_flange_pose_wrt_torso) {
    int planner_rtn_code, execute_return_code;
    ROS_INFO("planning hi-res move");
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_pose_wrt_torso);
    if (planner_rtn_code != cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("desired fine motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_MOVE_CARTESIAN_FINE;
    }
    //if succeed to here, send command to execute planned motion
    ROS_INFO("sending command to execute planned hi-res path:");
    execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    //assumes execution was successful...should return a more valuable code
    return object_grabber::object_grabberResult::SUCCESS;
}

//open-gripper func confirms opening before returning--or times out and complains
//"open" is defined by the filtered finger opening exceeding a provided threshold 

int ObjectGrabber::open_gripper(double open_val_test) {
    //open the gripper:
    ROS_INFO("opening gripper");
    double dt = 0.01;
    double stopwatch = 0.0;

    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>%f", baxterGripper.get_right_gripper_pos(), open_val_test);
    while ((baxterGripper.get_right_gripper_pos() < open_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        stopwatch += dt;
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(dt).sleep();
    }
    if (baxterGripper.get_right_gripper_pos() > open_val_test) {
        ROS_INFO("gripper is open to > %f", open_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_OPEN;
    } else {
        ROS_WARN("timeout expired without opening gripper");
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}

//close-gripper waits for finger separation to be less than test value,
// else times out

int ObjectGrabber::close_gripper(double close_val_test) {
    ROS_INFO("closing gripper");
    double dt = 0.01;
    double stopwatch = 0.0;
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    //g_right_gripper_pos=110;
    //ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    while ((baxterGripper.get_right_gripper_pos() > close_val_test)&&(stopwatch < GRIPPER_TIMEOUT)) {
        stopwatch += dt;
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        //ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(dt).sleep();
    }

    if (baxterGripper.get_right_gripper_pos() < close_val_test) {
        ROS_INFO("gripper is closed to < %f", close_val_test);
        return object_grabber::object_grabberResult::GRIPPER_IS_CLOSED;
    } else {
        ROS_WARN("timeout expired without closing gripper");
        ROS_INFO("gripper position is %f", baxterGripper.get_right_gripper_pos());
        return object_grabber::object_grabberResult::GRIPPER_FAILURE;
    }
}

//fnc uses tf to transform a provided pose into the torso frame

geometry_msgs::PoseStamped ObjectGrabber::convert_pose_to_system_ref_frame(geometry_msgs::PoseStamped pose_stamped) {
    //convert object pose into object pose w/rt torso:
    geometry_msgs::PoseStamped pose_stamped_wrt_sys_ref_frame;
    bool valid_tf = false;
    while (!valid_tf) {
        try {
            tfListener.transformPose("system_ref_frame", pose_stamped, pose_stamped_wrt_sys_ref_frame);
            valid_tf = true;
        } catch (tf::TransformException &ex) {
            ROS_WARN("transform not valid...retrying");
            valid_tf = false;
            ros::Duration(0.1).sleep();
        }
    }
    ROS_INFO("desired pose_stamped_wrt_sys_ref_frame_ origin: %f, %f, %f", pose_stamped_wrt_sys_ref_frame.pose.position.x,
            pose_stamped_wrt_sys_ref_frame.pose.position.y, pose_stamped_wrt_sys_ref_frame.pose.position.z);
    return pose_stamped_wrt_sys_ref_frame;
}



//fnc to compute gripper pose from block pose, per a specific grasp strategy
//obsolete: use object_to_flange_grasp_transform() instead
Eigen::Affine3d ObjectGrabber::block_grasp_transform(Eigen::Affine3d block_affine) {
    Eigen::Affine3d gripper_affine;
    //derive gripper approach pose from block pose:
    //compute a gripper pose with z-axis anti-parallel to object z-axis,
    // and x-axis coincident with object x-axis
    Eigen::Vector3d x_axis, y_axis, z_axis;
    Eigen::Matrix3d R_object, R_gripper;
    R_object = block_affine.linear(); //get 3x3 matrix from affine
    x_axis = R_object.col(0); //extract the x axis
    z_axis = -R_object.col(2); //define gripper z axis antiparallel to object z-axis
    y_axis = z_axis.cross(x_axis); // construct a right-hand coordinate frame
    R_gripper.col(0) = x_axis; //populate orientation matrix from axis directions
    R_gripper.col(1) = y_axis;
    R_gripper.col(2) = z_axis;
    //gripper_affine is defined to have origin coincident w/ block-frame origin, but z-axis antiparallel to block z-axis
    // and x-axis parallel to block-frame x-axis
    gripper_affine.linear() = R_gripper; //populate affine w/ orientation
    gripper_affine.translation() = block_affine.translation(); //and origin
    return gripper_affine;
}

//same as above, except uses stamped poses:
//obsolete: use object_to_flange_grasp_transform() instead
geometry_msgs::PoseStamped ObjectGrabber::block_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    Eigen::Affine3d block_affine, gripper_affine;
    geometry_msgs::PoseStamped gripper_pose;
    block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose);
    gripper_affine = block_grasp_transform(block_affine); // convert from block frame to gripper frame
    gripper_pose.header = block_pose.header;
    gripper_pose.pose = xformUtils.transformEigenAffine3dToPose(gripper_affine);
    return gripper_pose;
}

//does two steps: convert from block pose to gripper pose, then gripper pose to flange pose
// could use TF for 2nd step
//obsolete: use object_to_flange_grasp_transform() instead
geometry_msgs::PoseStamped ObjectGrabber::block_to_flange_grasp_transform(geometry_msgs::PoseStamped block_pose) {
    geometry_msgs::PoseStamped flange_pose_for_block_grasp;
    Eigen::Affine3d block_affine, gripper_affine, flange_affine;
    block_affine = block_affine = xformUtils.transformPoseToEigenAffine3d(block_pose.pose);
    gripper_affine = block_grasp_transform(block_affine); // convert from block frame to gripper frame
    flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange_.inverse(); // from gripper frame to flange frame
    flange_pose_for_block_grasp.header = block_pose.header; //and from affine to stamped pose
    flange_pose_for_block_grasp.pose = xformUtils.transformEigenAffine3dToPose(flange_affine);
    return flange_pose_for_block_grasp;
}

//more general--get grasp transform from object-properties library
//given object_id and given object's pose, compute corresponding right-arm toolflange pose for grasp
//input an object poseStamped and object_id; return a corresponding right-arm toolflange poseStamped for grasp

geometry_msgs::PoseStamped ObjectGrabber::object_to_flange_grasp_transform(int object_id, geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped flange_pose_for_object_grasp;
    Eigen::Affine3d object_affine, gripper_affine, flange_affine;
    Eigen::Vector3d origin;

    //get grasp_transform for this object ID: CAREFUL--need to make sure object_id is recognized!
    objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
            approach_dist_, gripper_test_val_);

    //convert object pose to an Affine 
    object_affine = xformUtils.transformPoseToEigenAffine3d(object_pose.pose);
    // given T_object/torso, want to compute T_flange/torso
    // use: (^torso)T_(object) = (^torso)T_(flange) * (^flange)T_(gripper) * (^gripper)T_(object)
    // (^gripper)T_(object) is grasp transform = desired object frame w/rt gripper frame
    // (^flange)T_(gripper) is fixed transform of gripper frame w/rt flange frame
    // given object frame w/rt torso, want to compute flange frame w/rt torso = (^torso)T_(flange)

    gripper_affine = object_affine * grasp_transform_.inverse();
    origin = gripper_affine.translation();
    ROS_INFO("gripper origin set to: %f, %f, %f", origin[0], origin[1], origin[2]);
    origin = a_right_gripper_frame_wrt_flange_.translation();
    ROS_INFO("gripperframe origin wrt flange: %f %f %f", origin[0], origin[1], origin[2]);

    //convert this back to a geometry_msgs PoseStamped object:
    flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange_.inverse(); // from gripper frame to flange frame
    origin = flange_affine.translation();
    ROS_INFO("flange origin wrt torso: %f %f %f", origin[0], origin[1], origin[2]);

    //flange_affine = gripper_affine * a_right_gripper_frame_wrt_flange_.inverse(); // from gripper frame to flange frame
    flange_pose_for_object_grasp.header = object_pose.header; //and from affine to stamped pose
    flange_pose_for_object_grasp.pose = xformUtils.transformEigenAffine3dToPose(flange_affine);
    return flange_pose_for_object_grasp; //and return the resulting computed flange pose
}


// a grasping function that assumes approach along the toolframe z-axis;
// does: opens gripper;
// performs a joint-space move to an approach pose, 
// moves with precision to the grasp pose along the tool-z direction
// closes the gripper
// does a Cartesian depart move, presumably holding the part

int ObjectGrabber::grasp_approach_tool_z_axis(geometry_msgs::PoseStamped des_flange_grasp_pose,
        double approach_dist, double gripper_close_test_val) {
    int rtn_val;
    //geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    geometry_msgs::PoseStamped des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_grasp_affine, flange_depart_affine;
    //convert right-gripper pose to an affine--SHOULD MAKE SURE POSE IS WRT TORSO!!
    flange_grasp_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_grasp_pose.pose);

    //the following coords are all right-arm toolflange frame w/rt torso
    Eigen::Vector3d approach_origin, flange_grasp_origin;
    flange_grasp_origin = flange_grasp_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;
    R = flange_grasp_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;

    a_flange_grasp_ = flange_grasp_affine; //copy to mem var
    // compute  a_flange_approach_, a_flange_depart_,
    a_flange_approach_ = a_flange_grasp_;
    //approach from distance dz_approach_offset_    
    a_flange_approach_.translation() = a_flange_grasp_.translation() - toolflange_z_axis*approach_dist;
    a_flange_depart_ = a_flange_approach_; //choose to depart to same pose as approach 
    ROS_INFO(" flange pose for object grasp: ");
    cout << "origin: " << a_flange_grasp_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_grasp_.linear() << endl;

    ROS_INFO("flange pose for approach: ");
    cout << "origin: " << a_flange_approach_.translation().transpose() << endl;
    cout << "R matrix: " << endl;
    cout << a_flange_approach_.linear() << endl;

    //open the gripper:
    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status; //failure to open gripper; return diagnostic
    }


    int move_to_rtn_code;
    //plan/execute Cartesian move to approach pose
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(a_flange_approach_);

    //move_to_rtn_code = move_flange_to(des_flange_approach_pose);
    //do jspace move instead to approach pose...may need to make approach higher
    move_to_rtn_code = jspace_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {

        return move_to_rtn_code; // give up--and send diagnostic code
    }

    // make a careful approach to grasp pose, along gripper-z direction (e.g., vertical)
    //plan path to grasp pose:
    move_to_rtn_code = fine_move_flange_to(des_flange_grasp_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code; // give up--and send diagnostic code
    }

    //close the gripper, hopefully to grasp the part
    gripper_status = close_gripper(gripper_close_test_val);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status; //failure to open gripper; return diagnostic
    }

    //ros::Duration(1).sleep(); //some extra settling time for grasp
    //if (baxterGripper.get_right_gripper_pos() < gripper_test_val) {
    //    ROS_WARN("object grabber: gripper closed too far; object not grasped");
    //    return object_grabber::object_grabberResult::FAILED_OBJECT_NOT_IN_GRIPPER;
    //}
    //depart vertically:
    des_flange_depart_pose = des_flange_approach_pose;
    move_to_rtn_code = move_flange_to(des_flange_depart_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        return move_to_rtn_code; // give up--and send diagnostic code
    }

    return object_grabber::object_grabberResult::SUCCESS;
}


//drop-off fnc;
// specify gripper pose for drop-off, so this can be used with objects of different heights and different grasp transforms;
// the parent pgm must compute the gripper pose that corresponds to desired object pose, taking into account grasp transform and
// object dimensions
//this func is more general than simply dropoff from above;  it is: approach dropoff location along
// tool z-axis, open gripper, depart along neg toolframe z-axis;  So, this COULD e.g. insert a peg
// into a hole w/ a tilted hole.  But expected most common use would be placement from above, using
// gravity as a fixture

int ObjectGrabber::dropoff_from_above(geometry_msgs::PoseStamped des_flange_dropoff_pose, double approach_dist) {
    int move_to_rtn_code;
    geometry_msgs::PoseStamped des_flange_approach_pose;
    Eigen::Affine3d dropoff_flange_affine, approach_flange_affine;
    //next two steps already handled by fnc block_to_flange_grasp_transform()
    //dropoff_gripper_affine = xformUtils.transformPoseToEigenAffine3d(dropoff_gripper_pose_wrt_torso.pose);
    //dropoff_flange_affine = dropoff_gripper_affine * a_right_gripper_frame_wrt_flange_.inverse();        
    dropoff_flange_affine = xformUtils.transformPoseToEigenAffine3d(des_flange_dropoff_pose.pose);


    //the following coords are all right-arm toolflange frame w/rt torso
    Eigen::Vector3d approach_flange_origin, dropoff_flange_origin;
    dropoff_flange_origin = dropoff_flange_affine.translation();
    Eigen::Vector3d toolflange_z_axis;
    Eigen::Matrix3d R;

    R = dropoff_flange_affine.linear();
    toolflange_z_axis = R.col(2);
    cout << "toolflange_z_axis: " << toolflange_z_axis.transpose() << endl;
    cout << "offset by " << approach_dist << " to descend to grasp pose" << endl;

    approach_flange_affine = dropoff_flange_affine; //derive flange pose for approach; start from drop-off pose
    //and offset in z-axis direction by -approach_dist
    approach_flange_affine.translation() = dropoff_flange_affine.translation() - toolflange_z_axis*approach_dist;

    //convert these back to stamped poses:
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(approach_flange_affine);
    des_flange_dropoff_pose.header.frame_id = "torso";
    des_flange_dropoff_pose.pose = xformUtils.transformEigenAffine3dToPose(dropoff_flange_affine);
    ROS_INFO("attempting move to approach pose");
    //move_to_rtn_code = move_flange_to(des_flange_approach_pose);
    move_to_rtn_code = jspace_move_flange_to(des_flange_approach_pose); //use jspace instead
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }
    //now do a careful approach move along gripper-z axis:
    ROS_INFO("attempting fine-move approach to drop-off");
    move_to_rtn_code = fine_move_flange_to(des_flange_dropoff_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }
    ros::Duration(0.5).sleep();
    //open the gripper:
    ROS_INFO("releasing the part");
    int gripper_status;
    gripper_status = open_gripper(95.0);
    if (object_grabber::object_grabberResult::GRIPPER_FAILURE == gripper_status) {
        return gripper_status; //failure to open gripper; return diagnostic
    }
    ros::Duration(0.5).sleep();
    //depart, carefully:
    ROS_INFO("computing/executing depart move");
    move_to_rtn_code = fine_move_flange_to(des_flange_approach_pose);
    if (move_to_rtn_code != object_grabber::object_grabberResult::SUCCESS) {
        ROS_WARN("failure: return code = %d", move_to_rtn_code);
        return move_to_rtn_code; // give up--and send diagnostic code
    }

    move_to_rtn_code = jspace_move_to_pre_pose();

    return object_grabber::object_grabberResult::SUCCESS;
}

//this fnc is intended to grasp a cylinder that is oriented vertically;
//approach w/ tool-flange z-axis parallel to table surface
// and with vector between fingertips parallel to table surface
// for Baxter gripper, this means orient right_gripper x-axis "up"
// thus can slide horizontally along tool-flange z-axis to approach the cylinder from the side

// assumes object origin is in the center of the cylinder, and z-axis is through cylinder axis of symmetry
// assumes fingers should close at z-height of object origin w/ fingertips diametrically opposed
// if cylinder is too short to allow for clearance, need to specify an additional vertical offset

// After grasping, depart normal to the table surface, e.g. as in picking up a bottle containing fluid
// NEEDS REVIEW: all frames for motion planning should be expressed w/respect to tool flange frame

//note--object_pose is already known to be w/rt torso, due to executeCB() vetting

int ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    geometry_msgs::PoseStamped des_flange_grasp_pose, des_flange_approach_pose, des_flange_depart_pose;
    Eigen::Affine3d flange_approach_affine, flange_grasp_affine, flange_depart_affine;

    int rtn_val, execute_return_code;
    //send a command to plan a joint-space move to pre-defined pose:   
    rtn_val = armMotionCommander.plan_move_to_pre_pose();
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path to pre-pose:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion to pre-pose is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }

    //inquire re/ right-arm joint angles:
    rtn_val = armMotionCommander.rt_arm_request_q_data();

    Eigen::Affine3d object_affine;
    object_affine =
            xformUtils.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    //grasp_origin_(2) = gripper_table_z_; //except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;
    // convert to flange pose:
    //NEED TO UPDATE THIS: GET GRASP FRAME FROM OBJECT_PROPERTIES LIBRARY
    flange_grasp_affine = a_gripper_grasp_ * a_right_gripper_frame_wrt_flange_.inverse();

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    flange_approach_affine = flange_grasp_affine;
    flange_approach_affine.translation() = flange_grasp_affine.translation() - gripper_b_des_*L_approach_;

    // after have cylinder grasped, move purely upwards by z_depart:
    flange_depart_affine = flange_grasp_affine;
    flange_depart_affine.translation() = flange_grasp_affine.translation() + gripper_n_des_*z_depart_;

    //open the gripper:
    ROS_INFO("opening gripper");
    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() < 95.0) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }

    //start w/ a jnt-space move from current pose to approach pose:
    int planner_rtn_code;
    des_flange_approach_pose.header.frame_id = "torso";
    des_flange_approach_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_approach_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_approach_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }

    //slide to can:
    ROS_INFO("planning hi-res approach to grasp pose");
    des_flange_grasp_pose.header.frame_id = "torso";
    des_flange_grasp_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_grasp_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_fine_path_current_to_goal_flange_pose(des_flange_grasp_pose);
    //optionally, rescale this path to slow it down:
    double time_stretch_factor = 3.0; // tune this approach speed; e.g., slow down by factor of 3
    planner_rtn_code = armMotionCommander.rt_arm_timestretch_planned_path(time_stretch_factor);

    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }


    //close the gripper:
    ROS_INFO("closing gripper");
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    //g_right_gripper_pos=110;
    ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() > 90.0) {
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }
    ros::Duration(1).sleep(); //some extra settling time for grasp

    //depart vertically:
    des_flange_depart_pose.header.frame_id = "torso";
    des_flange_depart_pose.pose = xformUtils.transformEigenAffine3dToPose(flange_depart_affine);
    planner_rtn_code = armMotionCommander.rt_arm_plan_path_current_to_goal_flange_pose(des_flange_depart_pose);
    if (planner_rtn_code == cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        //send command to execute planned motion
        ROS_INFO("sending command to execute planned path:");
        execute_return_code = armMotionCommander.rt_arm_execute_planned_path();
    } else {
        ROS_WARN("desired motion is not feasible");
        return object_grabber::object_grabberResult::FAILED_CANNOT_REACH;
    }
    return object_grabber::object_grabberResult::SUCCESS;

}


//can access various low-level behaviors, e.g. MOVE_TO_PRE_POSE, OPEN_GRIPPER, CLOSE_GRIPPER
//useful high-level cases are:
//GRAB_W_TOOL_Z_APPROACH and DROPOFF_ALONG_TOOL_Z, which combine multiple steps of motion planning
// and execution, coordinated with gripper actions,  based on a strategy of approach along 
// the toolframe z-axis, customized for the current object_id
// 
// case GRAB_UPRIGHT_CYLINDER is a variation (that needs refactoring and testing); it also approaches
// grasp along tool-z, but the approach direction will be horizontal, followed by a lift that is
// vertical, e.g. to grasp and lift a container of fluid without spilling 

void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {

    int action_code = goal->action_code;
    ROS_INFO("got action code %d", action_code);
    int object_grabber_rtn_code;
    switch (action_code) {
        case object_grabber::object_grabberGoal::GRAB_UPRIGHT_CYLINDER:
            ROS_INFO("case GRAB_UPRIGHT_CYLINDER");
            object_pose_stamped_ = goal->desired_frame;
            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);

            object_grabber_rtn_code = vertical_cylinder_power_grasp(object_pose_stamped_wrt_torso_);
            grab_result_.return_code = object_grabber_rtn_code;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        case object_grabber::object_grabberGoal::GRAB_W_TOOL_Z_APPROACH: //more general name
            ROS_INFO("case GRAB_W_TOOL_Z_APPROACH");
            object_pose_stamped_ = goal->desired_frame;
            object_id_ = goal->object_id; // e.g. TOY_BLOCK_ID; 
            ROS_INFO("object_id = %d", object_id_);

            //look up object manipulation properties from library:
            if (!objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
                    approach_dist_, gripper_test_val_)) {
                ROS_WARN("object ID not recognized");
                grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
                object_grabber_as_.setAborted(grab_result_);
            }

            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);
            des_flange_pose_stamped_wrt_torso_ = object_to_flange_grasp_transform(object_id_,
                    object_pose_stamped_wrt_torso_);

            object_grabber_rtn_code = grasp_approach_tool_z_axis(des_flange_pose_stamped_wrt_torso_,
                    approach_dist_, gripper_test_val_);

            grab_result_.return_code = object_grabber_rtn_code;
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
            
            //DROPOFF_ALONG_TOOL_Z means: approach dropoff pose along toolflange z-axis, then
            // open gripper, then depart along neg toolframe z axis back
            // approach/depart distance is param associated with object_manipulation_properties
        case object_grabber::object_grabberGoal::DROPOFF_ALONG_TOOL_Z: //more general name
            //case object_grabber::object_grabberGoal::PLACE_TOY_BLOCK: //specific example
            ROS_INFO("case DROPOFF_ALONG_TOOL_Z");
            object_pose_stamped_ = goal->desired_frame; //make sure is expressed w/rt torso frame
            object_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(object_pose_stamped_);
            object_id_ = goal->object_id; //e.g. TOY_BLOCK_ID; 
            //look up object manipulation properties from library:
            if (!objectManipulationProperties.get_object_info(object_id_, grasp_transform_,
                    approach_dist_, gripper_test_val_)) {
                ROS_WARN("object ID not recognized");
                grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
                object_grabber_as_.setAborted(grab_result_);
            }

            //need to consider grasp transform, from object frame to flange frame
            des_flange_pose_stamped_wrt_torso_ = object_to_flange_grasp_transform(object_id_,
                    object_pose_stamped_wrt_torso_);
            //des_flange_pose_stamped_wrt_torso_ = block_to_flange_grasp_transform(object_pose_stamped_wrt_torso_);

            object_grabber_rtn_code = dropoff_from_above(des_flange_pose_stamped_wrt_torso_, approach_dist_);
            grab_result_.return_code = object_grabber_rtn_code;
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
            //partial move commands--lower level than above; interpret object_pose_stamped_wrt_torso_ as desired FLANGE pose
        case object_grabber::object_grabberGoal::MOVE_FLANGE_TO:
            ROS_INFO("case MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        case object_grabber::object_grabberGoal::JSPACE_MOVE_FLANGE_TO:
            ROS_INFO("object-grabber: case JSPACE_MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = jspace_move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;

            break;

        case object_grabber::object_grabberGoal::FINE_MOVE_FLANGE_TO:
            ROS_INFO("case FINE_MOVE_FLANGE_TO");
            des_flange_pose_stamped_ = goal->desired_frame;
            des_flange_pose_stamped_wrt_torso_ = convert_pose_to_torso_frame(des_flange_pose_stamped_);
            grab_result_.return_code = fine_move_flange_to(des_flange_pose_stamped_wrt_torso_);
            grab_result_.des_flange_pose_stamped_wrt_torso = des_flange_pose_stamped_wrt_torso_;
            object_grabber_as_.setSucceeded(grab_result_); //"succeeded" just means goal was processed; need to inspect rtn code to see result
            break;
        case object_grabber::object_grabberGoal::OPEN_GRIPPER:
            open_gripper(95.0); //hard coded; want gripper to be almost fully open
            break;
        case object_grabber::object_grabberGoal::CLOSE_GRIPPER:
            //ASSUMES gripper_test_val_ has been set by a call to object_manipulation_properties
            ROS_INFO("closing gripper to test val %f", gripper_test_val_);
            close_gripper(gripper_test_val_);
            break;
            //need ability to get hand out of way of camera
        case object_grabber::object_grabberGoal::MOVE_TO_PRE_POSE:
            grab_result_.return_code = jspace_move_to_pre_pose();
            object_grabber_as_.setSucceeded(grab_result_);
            break;

        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabberResult::ACTION_CODE_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }

}

Eigen::Affine3d ObjectGrabber::get_right_tool_transform(void) {
    //get transform from right_gripper to right_hand

    ROS_INFO("listening for gripper-to-toolflange transform:");
    tf::StampedTransform stf_gripper_wrt_flange;
    bool tferr = true;
    ROS_INFO("waiting for tf between right gripper and right tool flange...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("right_hand", "right_gripper", ros::Time(0), stf_gripper_wrt_flange);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("right gripper to right tool flange tf is good");
    xformUtils.printStampedTf(stf_gripper_wrt_flange);
    tf::Transform tf_kinect_wrt_base = xformUtils.get_tf_from_stamped_tf(stf_gripper_wrt_flange);
    Eigen::Affine3d affine_gripper_wrt_flange = xformUtils.transformTFToAffine3d(tf_kinect_wrt_base);

    std::cout << "affine rotation: " << std::endl;
    std::cout << affine_gripper_wrt_flange.linear() << std::endl;
    std::cout << "affine offset: " << affine_gripper_wrt_flange.translation().transpose() << std::endl;
    return affine_gripper_wrt_flange;
}
 */