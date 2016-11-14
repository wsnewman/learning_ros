//define ObjectGrabber class; contains functionality to
// serve manipulation goals
#ifndef OBJECT_GRABBER_H
#define	OBJECT_GRABBER_H
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabber3Action.h>
#include <cartesian_planner/cart_moveAction.h>
#include <cartesian_planner/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/gripper_ID_codes.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <object_manipulation_properties/objectManipulationQuery.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include<generic_gripper_services/genericGripperInterface.h>
class ObjectGrabber {
private:
    ros::NodeHandle nh_;
    XformUtils xformUtils;
    ArmMotionCommander arm_motion_commander_; //robot-independent class to interact w/ cartesian-moves action server 
    ros::ServiceClient manip_properties_client_; // = n.serviceClient<object_manipulation_properties::objectManipulationQuery>("object_manip_query_svc");
    object_manipulation_properties::objectManipulationQuery manip_properties_srv_;

    ros::ServiceClient gripper_client_; //generic gripper interface
    generic_gripper_services::genericGripperInterface gripper_srv_;

    //messages to send/receive cartesian goals / results:
    object_grabber::object_grabber3Goal grab_goal_;
    object_grabber::object_grabber3Result grab_result_; 
    object_grabber::object_grabber3Feedback grab_fdbk_;    
    geometry_msgs::PoseStamped object_pose_stamped_;
    //pose of object w/rt generic_gripper_frame for grasp, approach, depart:
    geometry_msgs::PoseStamped grasp_pose_,approach_pose_,depart_pose_,dropoff_pose_;
    //from object_manip service, defines poses as:
    // where is the object w/rt the gripper for: approach, grasp, depart
    // separate calls for GET_GRASP_POSE_TRANSFORMS, GET_APPROACH_POSE_TRANSFORMS, GET_DEPART_POSE_TRANSFORMS
    geometry_msgs::Pose grasp_object_pose_wrt_gripper_;
    geometry_msgs::Pose approach_object_pose_wrt_gripper_; //from gripper viewpoint, where is object
                                                                  //when gripper is at approach pose for current
                                                                  //grasp strategy
    geometry_msgs::Pose depart_object_pose_wrt_gripper_;   //departure strategy depends on how object
                                                                  //is grasped and how want to lift it
                                                                  
    //geometry_msgs::PoseStamped lift_object_pose_wrt_gripper_; // depart for lifting object can be different
    //geometry_msgs::PoseStamped withdraw_object_pose_wrt_gripper_; //from depart after releasing object

    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_; 
    double computed_arrival_time_;
    
    
    int object_code_;
    std_msgs::Bool grasp_,release_;
  
    int gripper_id_;
    
    Eigen::Vector3d gripper_b_des_;
    Eigen::Vector3d gripper_n_des_;
    Eigen::Vector3d gripper_t_des_;
    Eigen::Vector3d grasp_origin_,approach_origin_,depart_origin_;
    Eigen::Matrix3d R_gripper_vert_cyl_grasp_;
    Eigen::Affine3d a_gripper_start_,a_gripper_end_;
    Eigen::Affine3d a_gripper_approach_,a_gripper_depart_, a_gripper_grasp_;
    Eigen::Affine3d a_gripper_frame_wrt_flange;

    ros::Publisher gripper_publisher_;
 //want these functions:
    //int32 MOVE_TO_WAITING_POSE = 1 #move to a pose, defined on param server, that is convenient
                               //#e.g., prepared to approach a surface, but out of way of sensors
//int32 PLAN_MOVE_TO_GRASP_POSE  =2   #expects parameters of current_object_pose, object_ID, grasp_option, approach_option
                               //#must send separate "grasp" command to gripper
//int32 PLAN_MOVE_FINE_TO_GRASP_POSE  =3   #as above, but finer/slower approach motion
//int32 PLAN_MOVE_OBJECT_JSPACE =4    #move a grasped object to a destination pose using simple, joint-space move
                               //#expects params: des_object_pose, object_ID, grasp_option (need to know how object is grasped)
//int32 PLAN_MOVE_OBJECT_CSPACE = 5   #move a grasped object with Cartesian motion to a desired object pose
                               //#params:  des_object_pose, object_ID, grasp_option
//int32 PLAN_MOVE_FINE_OBJECT_CSPACE = 6 #as above, but w/ finer, slower motion

//int32 PLAN_WITHDRAW_FROM_OBJECT = 7 #with object grasp released, perform departure from object using specified depart strategy
                               //#params: object_ID, grasp_option, depart_option
//int32 PLAN_WITHDRAW_FINE_FROM_OBJECT = 8 #as above, but slower/more precise motion

//int32 PLAN_OBJECT_GRASP = 9  #combine multiple elements above to acquire an object

//int32 SET_SPEED_FACTOR = 10    #use arg speed_factor to change time scale of trajectory plan; larger than 1.0--> slower

            
    actionlib::SimpleActionServer<object_grabber::object_grabber3Action> object_grabber_as_;
    actionlib::SimpleActionClient<cartesian_planner::cart_moveAction> cart_move_action_client_;
    void cartMoveDoneCb_(const actionlib::SimpleClientGoalState& state,
        const cartesian_planner::cart_moveResultConstPtr& result);
    //this is a complex fnc that interacts with the object_manipulation_query_svc
    // get get grasp poses/strategies given a gripper_id and object_id
    //bool set_gripper_transforms(int gripper_id,int object_id, geometry_msgs::PoseStamped &grasp_pose,
    //  geometry_msgs::PoseStamped &approach_pose,geometry_msgs::PoseStamped &depart_pose);      
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabber3Action>::GoalConstPtr& goal);  
    bool get_gripper_id();
    bool get_default_grab_poses(int object_id,geometry_msgs::PoseStamped object_pose_stamped);   
    bool get_default_dropoff_poses(int object_id,geometry_msgs::PoseStamped object_dropoff_pose_stamped);
    int grab_object(int object_id,geometry_msgs::PoseStamped object_pose_stamped);   
    int dropoff_object(int object_id,geometry_msgs::PoseStamped desired_object_pose_stamped);
    
    //void vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose);    
    //void grasp_from_approach_pose(geometry_msgs::PoseStamped approach_pose, double approach_dist);

public:

        ObjectGrabber(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
        //void set_tool_xform(Eigen::Affine3d xform) { a_gripper_frame_wrt_flange = xform; };

    ~ObjectGrabber(void) {
    }
    //define some member methods here

};

#endif