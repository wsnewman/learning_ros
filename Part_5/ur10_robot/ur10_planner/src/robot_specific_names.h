//put robot-specific names here:
string g_urdf_base_frame_name("base_link");
string g_urdf_flange_frame_name("tool0");
string g_joint_states_topic_name("/joint_states");
string g_traj_pub_topic_name("joint_path_command");
string g_traj_as_name("/arm_controller/follow_joint_trajectory");
std::vector<std::string> g_jnt_names{"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
bool g_use_trajectory_action_server = true;
