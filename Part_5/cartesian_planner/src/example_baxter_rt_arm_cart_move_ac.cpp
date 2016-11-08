// example_baxter_rt_arm_cart_move_ac: 
// wsn, Nov, 2016
// illustrates use of baxter_rt_arm_cart_move_as, action server called "cartMoveActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
XformUtils xformUtils;

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmMotionCommander {
private:
    ros::NodeHandle nh_;

    //messages to send/receive cartesian goals / results:
    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_;    
    std::vector <double> q_vec_; //holder for right-arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<cartesian_planner::cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,
    const cartesian_planner::cart_moveResultConstPtr& result);
public:
        ArmMotionCommander(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionCommander(void) {
    }
    void send_test_goal(void);
    int plan_move_to_pre_pose(void);
    int execute_planned_path(void);
    int request_q_data(void);
    int request_tool_pose(void);
    geometry_msgs::PoseStamped get_tool_pose_stamped(void) { return tool_pose_stamped_;};
    
    Eigen::VectorXd get_joint_angles(void); 
    int plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec);  
    int plan_path_current_to_goal_gripper_pose(geometry_msgs::PoseStamped des_pose);
    int plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);

    //utilities to convert between affine and pose; use xformUtils instead
    //Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose); 
    //geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

};

ArmMotionCommander::ArmMotionCommander(ros::NodeHandle* nodehandle): nh_(*nodehandle),
cart_move_action_client_("cartMoveActionServer", true) { // constructor
    ROS_INFO("in constructor of ArmMotionInterface");

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
    
}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
        const cartesian_planner::cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_=*result;
}

/* use xformUtilsinstead
Eigen::Affine3d ArmMotionCommander::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}

geometry_msgs::Pose ArmMotionCommander::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}
*/

void ArmMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } else {
            ROS_INFO("finished before timeout");
            ROS_INFO("return code: %d",cart_result_.return_code);
        }        
}

int ArmMotionCommander::plan_move_to_pre_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_WAITING_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {    
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    cart_goal_.q_goal.resize(7);
    for (int i=0;i<7;i++) cart_goal_.q_goal[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;    
    
}

int ArmMotionCommander::plan_path_current_to_goal_gripper_pose(geometry_msgs::PoseStamped des_pose) {
    
    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_GRIPPER_POSE;
    ROS_INFO("des_pose: ");
    xformUtils.printStampedPose(des_pose);
    cart_goal_.des_pose_gripper = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;        
}

int ArmMotionCommander::plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {
    
    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;      
}
    

int ArmMotionCommander::execute_planned_path(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
int ArmMotionCommander::request_q_data(void) {
   ROS_INFO("requesting right-arm joint angles");
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }
    
    q_vec_ = cart_result_.q_arm;
    ROS_INFO("move returned success; right arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f; %f",q_vec_[0],q_vec_[1],q_vec_[2],q_vec_[3],q_vec_[4],q_vec_[5],q_vec_[6]);
    return (int) cart_result_.return_code;
}

Eigen::VectorXd ArmMotionCommander::get_joint_angles(void) {
    request_q_data();
    Eigen::VectorXd angs_vecXd;
    angs_vecXd.resize(7);
    for (int i=0;i<7;i++) {
        angs_vecXd[i] = q_vec_[i];
    }
    return angs_vecXd;
}

int ArmMotionCommander::request_tool_pose(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo torso yale_gripper_frame
    ROS_INFO("requesting right-arm tool pose");    
    cart_goal_.command_code = cartesian_planner::cart_moveGoal::GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }    
    
        tool_pose_stamped_ = cart_result_.current_pose_gripper;
        ROS_INFO("move returned success; right arm tool pose: ");
        ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
                tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
        ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
                tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
                tool_pose_stamped_.pose.orientation.w);
  return (int) cart_result_.return_code;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_baxter_rt_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    ArmMotionCommander arm_motion_commander(&nh);
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    geometry_msgs::PoseStamped tool_pose;
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("commanding move to waiting pose");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    rtn_val=arm_motion_commander.request_tool_pose();
    
    //do a joint-space move; get the start angles:
    joint_angles = arm_motion_commander.get_joint_angles();
    
    //increment all of the joint angles by a fixed amt:
    for (int i=0;i<7;i++) joint_angles[i]+=0.2;
    ROS_INFO("joint-space move, all joints +0.2 rad");
    //try planning a joint-space motion to this new joint-space pose:
    rtn_val=arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);

    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.request_q_data();
    
    //return to pre-defined pose:
    ROS_INFO("back to waiting pose");
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    rtn_val=arm_motion_commander.execute_planned_path();    

    //get tool pose
    rtn_val = arm_motion_commander.request_tool_pose();
    tool_pose = arm_motion_commander.get_tool_pose_stamped();
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose);
    //alter the tool pose:
    std::cout<<"enter 1: ";
    int ans;
    std::cin>>ans;
    //tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("planning Cartesian move to goal pose w/ dpx = 0.2, dpy =0.2");
    //tool_pose.pose.position.y -= 0.2; // move 20cm, along y in torso frame
    tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=arm_motion_commander.plan_path_current_to_goal_gripper_pose(tool_pose);
    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();
    
    //try vector cartesian displacement at fixed orientation:
    ROS_INFO("will plan vertical motion");
    std::cout<<"enter desired delta-z: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    rtn_val = arm_motion_commander.plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=arm_motion_commander.execute_planned_path();
    }
    return 0;
}

