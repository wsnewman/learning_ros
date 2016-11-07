//example client of object_manip_query_svc:
// first run: rosrun object_manipulation_properties object_manipulation_query_svc 
// then start this node:  
// rosrun object_manipulation_properties example_object_manip_query_client

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h> //these could be handy in the future
#include <geometry_msgs/PoseStamped.h>
#include <object_manipulation_properties/gripper_ID_codes.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <object_manipulation_properties/objectManipulationQuery.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_manip_query_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<object_manipulation_properties::objectManipulationQuery>("object_manip_query_svc");
    object_manipulation_properties::objectManipulationQuery srv;
    XformUtils xformUtils;
    int gripper_id;
    int object_id;
    int grasp_option;
    int query_code;
    int n_options;
    vector<geometry_msgs::Pose> grasp_pose_options;
    
    while (ros::ok()) {
        cout<<endl;
        cout << "enter a gripper code: (e.g. 4 for RETHINK_ELECTRIC_GRIPPER_RT, or -1 to quit): ";
        cin>>gripper_id;
        if (gripper_id<0)
            return 0;
        srv.request.gripper_ID = gripper_id; 
        
        cout << "enter an object ID code (e.g. 1000 for TOY_BLOCK): ";
        cin>>object_id;        
        srv.request.object_ID = object_id; 
        
        cout << "enter a query code (e.g. 1 for GRASP_STRATEGY_OPTIONS_QUERY): ";
        cin>>query_code;  
        srv.request.query_code = query_code;
        //2 cases--asking for option codes, or asking for a specific pose: GET_GRASP_POSE_TRANSFORMS
        if (query_code < object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS) {
            client.call(srv);
            if (!srv.response.valid_reply) {
                ROS_WARN("inquiry not valid");
            }
            else {
                n_options= srv.response.grasp_strategy_options.size();
                if (n_options<1) {
                    ROS_WARN("no grasp strategy options known for this case");
                }
                else {
                    for (int i=0;i<n_options;i++) {
                        ROS_INFO("grasp strategy option %d is: %d",i,srv.response.grasp_strategy_options[i]);
                    }
                }
            }
        }
        else { //try asking for a specific code:
            cout<<"enter grasp-strategy option code: ";
            cin>>grasp_option;
            srv.request.grasp_option = grasp_option;
            client.call(srv);
            if (!srv.response.valid_reply) {
                ROS_WARN("inquiry not valid");
            }
            else {
                int n_pose_options = srv.response.gripper_pose_options.size();
                if (n_pose_options<1) {
                    ROS_WARN("no pose options returned for this case");
                }
                else {
                  grasp_pose_options = srv.response.gripper_pose_options;
                  ROS_INFO("gripper pose options: ");
                  for (int i=0;i<n_pose_options;i++) {
                      xformUtils.printPose(grasp_pose_options[i]);
                  }
                }
                
            }
        }
    }
    return 0;
}
