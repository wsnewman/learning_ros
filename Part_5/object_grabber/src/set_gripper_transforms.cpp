//fnc to interact with object manipulation query service to choose poses for approach, grasp and depart
// for specified gripper and object;
// this function could be smarter, including evaluating feasibility and move time for alternative
// grasp strategies.  In this example, simply choose the first available option
bool set_gripper_transforms(ros::NodeHandle &nh,int gripper_id,int object_id, geometry_msgs::PoseStamped &grasp_pose,
    geometry_msgs::PoseStamped &approach_pose,geometry_msgs::PoseStamped &depart_pose) {
    int grasp_option=0;
    int approach_strategy;
    int depart_strategy;
    int query_code;
    int n_options;
    vector<geometry_msgs::Pose> grasp_pose_options;
    //all poses will be expressed as pose of object w/rt generic_gripper_frame
    //generic_gripper_frame should be defined for all gripper options
    grasp_pose.header.frame_id = "generic_gripper_frame";
    approach_pose.header.frame_id = "generic_gripper_frame";   
    depart_pose.header.frame_id = "generic_gripper_frame";     
    ros::ServiceClient manip_query_client = nh.serviceClient<object_manipulation_properties::objectManipulationQuery>("object_manip_query_svc");
    //try to connect to service:
    while (!manip_query_client.exists()) {
      ROS_INFO("waiting for object_manip_query_svc...");
      ros::Duration(0.1).sleep();
    }
    ROS_INFO("connected client to service");       

    object_manipulation_properties::objectManipulationQuery manip_query_srv;    
    manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
    manip_query_srv.request.gripper_ID = gripper_id; 
    manip_query_srv.request.object_ID = object_id; 
    manip_query_srv.request.grasp_option = grasp_option; //don't need this for first call
 
    manip_query_client.call(manip_query_srv);
    if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("manipulation inquiry not valid");
                return false;
            } 
    //if here, got valid reply from query service
    n_options= manip_query_srv.response.grasp_strategy_options.size();
  if (n_options<1) {
                    ROS_WARN("no grasp strategy options known for this case");
                    ROS_WARN("gripper id: %d, object id: %d",gripper_id,object_id);
                    return false; // nothing more can be done
                }
  //if here, have options:
  for (int i=0;i<n_options;i++) {
      ROS_INFO("grasp strategy option %d is: %d",i,manip_query_srv.response.grasp_strategy_options[i]);
  }
  //arbitrarily, choose the first grasp-strategy option; 
  //more generally, could evaluate options and choose one
  grasp_option =  manip_query_srv.response.grasp_strategy_options[0];
  //get the pose of the object w/rt gripper frame for this object, this gripper and this grasp option:
  manip_query_srv.request.grasp_option = grasp_option;
  manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;
  manip_query_client.call(manip_query_srv);
  
  if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("inquiry not valid");
                return false; //give up
            }
  //else:
  int n_pose_options = manip_query_srv.response.gripper_pose_options.size();
  if (n_pose_options<1) {
                    ROS_WARN("no pose options returned for this case");
                    return false; // give up
                }
  //else:
  grasp_pose_options = manip_query_srv.response.gripper_pose_options;
  ROS_INFO("grasp pose options: ");
                  for (int i=0;i<n_pose_options;i++) {
                      xformUtils.printPose(grasp_pose_options[i]);
                  }
  //arbitrarily choose the first option; better would be to evaluate options and choose optimally
  grasp_pose.pose = grasp_pose_options[0];
  grasp_pose.header.frame_id = "generic_gripper_frame";
  
  //get approach pose; inquire what approach options are available:
  manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
    manip_query_client.call(manip_query_srv);
    if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("could not find valid approach option; manipulation inquiry not valid");
                return false;
            }   
    //arbitrarily choose the first option:
    approach_strategy = manip_query_srv.response.grasp_strategy_options[0];
    // find gripper pose for this approach strategy:
    manip_query_srv.request.grasp_option = approach_strategy;
   manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
    manip_query_client.call(manip_query_srv);
    if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("could not find valid approach option; manipulation inquiry not valid");
                return false;
            }   
  grasp_pose_options = manip_query_srv.response.gripper_pose_options;
  ROS_INFO("approach pose options: ");
                  for (int i=0;i<n_pose_options;i++) {
                      xformUtils.printPose(grasp_pose_options[i]);
                  }
  //arbitrarily choose the first option; better would be to evaluate options and choose optimally
  approach_pose.pose = grasp_pose_options[0];    

  //finally, set the depart pose:
  manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
    manip_query_client.call(manip_query_srv);
    if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("could not find valid approach option; manipulation inquiry not valid");
                return false;
            }   
    //arbitrarily choose the first option:
    depart_strategy = manip_query_srv.response.grasp_strategy_options[0];
    // find gripper pose for this depart strategy:
    manip_query_srv.request.grasp_option = depart_strategy;
    manip_query_srv.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
    manip_query_client.call(manip_query_srv);
    if (!manip_query_srv.response.valid_reply) {
                ROS_WARN("could not find valid depart option; manipulation inquiry not valid");
                return false;
            }   
  grasp_pose_options = manip_query_srv.response.gripper_pose_options;
  ROS_INFO("depart pose options: ");
                  for (int i=0;i<n_pose_options;i++) {
                      xformUtils.printPose(grasp_pose_options[i]);
                  }
  //arbitrarily choose the first option; better would be to evaluate options and choose optimally  
  depart_pose.pose = grasp_pose_options[0]; 
  return true; // made it to here--> all is well
}

