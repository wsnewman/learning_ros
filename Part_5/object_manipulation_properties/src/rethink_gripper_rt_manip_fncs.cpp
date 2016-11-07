//MANIPULATOR OPTIONS FOR RETHINK GRIPPER:
//for each combination of gripper/object, need a function like this:



//provide details for grasping object TOY_BLOCK using RETHINK gripper
// need one of these specialized functions for every case of object_ID for RETHINK gripper

void rethink_grasp_TOY_BLOCK_ID(int query_code, int grasp_option,
        object_manipulation_properties::objectManipulationQueryResponse&response) {

    std::vector<geometry_msgs::Pose> object_grasp_poses_wrt_gripper;
    std::vector<geometry_msgs::Pose> object_approach_poses_wrt_gripper;
    std::vector<geometry_msgs::Pose> object_depart_poses_wrt_gripper;
    geometry_msgs::Pose object_pose_wrt_gripper;
    Eigen::Matrix3d R_object_wrt_gripper;
    Eigen::Vector3d object_origin_wrt_gripper_frame;
    Eigen::Vector3d x_axis, y_axis, z_axis;
    XformUtils xformUtils;
    ROS_INFO("query, baxter gripper, toy_block; query code %d, grasp option %d", query_code, grasp_option);
    //here are two grasp poses using baxter gripper to grasp toy_block:
    object_pose_wrt_gripper.position.x = 0.0; //easy--align gripper origin w/ object origin
    object_pose_wrt_gripper.position.y = 0.0;
    object_pose_wrt_gripper.position.z = 0.0;
    object_pose_wrt_gripper.orientation.x = 1.0; //x-axis parallel, z-axis anti-parallel:
    object_pose_wrt_gripper.orientation.y = 0.0; //grasp from above
    object_pose_wrt_gripper.orientation.z = 0.0;
    object_pose_wrt_gripper.orientation.w = 0.0;
    object_grasp_poses_wrt_gripper.clear();
    object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
    object_pose_wrt_gripper.orientation.x = 0.0; //x-axis anti-parallel, z-axis anti-parallel:
    object_pose_wrt_gripper.orientation.y = 1.0; //grasp from above     
    object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
    Eigen::Affine3d affine_object_wrt_gripper, affine_object_wrt_gripper_approach, affine_object_wrt_gripper_depart;
    //affine_object_wrt_gripper = xformUtils.transformPoseToEigenAffine3d(geometry_msgs::Pose pose);   

    switch (query_code) {
        case object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY:
            //respond to query for grasp options
            response.grasp_strategy_options.clear();
            //in this case, the only option specified is grasp-from-above--which really means
            // from object z-direction...still works if object pose is tilted
            // only alternative, at present, is GRASP_FROM_SIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::GRASP_FROM_ABOVE);
            //this version does not populate tolerances of grasp;
            response.valid_reply = true;
            break;
        case object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY:
            response.grasp_strategy_options.clear();
            //only options, at present, are APPROACH_Z_TOOL and APPROACH_LATERAL_SLIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::APPROACH_Z_TOOL);
            response.valid_reply = true;
            break;
        case object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY:
            response.grasp_strategy_options.clear();
            //only options, at present, are DEPART_Z_TOOL and DEPART_LATERAL_SLIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::DEPART_Z_TOOL);
            response.valid_reply = true;
            break;
            //inquiry for grasp transform w/ specified grasp option:
        case object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS:
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::GRASP_FROM_ABOVE) {
                //fill in grasp pose: pose of object frame w/rt gripper frame
                //geometry_msgs/Pose[] gripper_pose_options
                response.gripper_pose_options.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                //return the two grasp-pose options
                response.valid_reply = true;
                break;
            } else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_ID");
                response.valid_reply = false;
                break;
            }

        case object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS:
            ROS_INFO("get approach pose transforms...");
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::APPROACH_Z_TOOL) {
                //compute the approach transform, i.e. pose of object w/rt gripper in approach pose from above
                ROS_INFO("approach grasp along tool-z direction: ");
                object_approach_poses_wrt_gripper.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
                    object_pose_wrt_gripper.position.z += 0.1; // add approach dist in gripper z direction    
                    object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
                    response.gripper_pose_options.clear();

                }
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                response.valid_reply = true;
                break;
            } else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_ID");
                response.valid_reply = false;
                break;
            }
        case object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS:
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::DEPART_Z_TOOL) {
                //compute the depart transform, i.e. pose of object w/rt gripper to depart above
                //in this case, identical to approach poses
                ROS_INFO("depart strategy along -tool-z direction");
                object_approach_poses_wrt_gripper.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
                    object_pose_wrt_gripper.position.z += 0.1; // add approach dist in gripper z direction    
                    object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
                }
                response.gripper_pose_options.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                response.valid_reply = true;
                break;
            } else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object TOY_BLOCK_ID");
                response.valid_reply = false;
                break;
            }

        default:
            ROS_WARN("grasp poses for rethink gripper unknown for object code TOY_BLOCK_ID");
            response.valid_reply = false;
            break;
    }

}

void rethink_grasp_query(int object_id, int query_code, int grasp_option,
        object_manipulation_properties::objectManipulationQueryResponse&response) {
    switch (object_id) {
        case ObjectIdCodes::TOY_BLOCK_ID:
            rethink_grasp_TOY_BLOCK_ID(query_code, grasp_option, response);
            break;
            //case OBJECT_ID: add more cases here, for each object to be grasped by rethink gripper
            //need a separate function to fill response for each object/gripper combo
        default:
            response.valid_reply = false;
            break;
    }
}
