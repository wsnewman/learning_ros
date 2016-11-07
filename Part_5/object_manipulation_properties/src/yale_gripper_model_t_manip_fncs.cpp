//MANIPULATOR OPTIONS FOR YALE MODEL-T GRIPPER:
//for each combination of gripper/object, need a function like this:

void yale_gripper_model_t_grasp_query(int object_id, int query_code, int grasp_option,
        object_manipulation_properties::objectManipulationQueryResponse&response) {
    switch (object_id) {
        //case TOY_BLOCK_ID:
        //    rethink_grasp_TOY_BLOCK_ID(query_code, grasp_option, response);
        //    break;
            //case OBJECT_ID: add more cases here, for each object to be grasped by rethink gripper
            //need a separate function to fill response for each object/gripper combo
        default:
            response.valid_reply = false;
            break;
    }
}


