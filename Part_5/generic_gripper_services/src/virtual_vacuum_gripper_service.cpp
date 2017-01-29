//virtual_vacuum_gripper_service.cpp
//wsn Nov 2016
//provides a service to process gripper options, intended to use a generic gripper interface,
//but  interfaces to a specific gripper--e.g. sticky-fingers virtual vacuum gripper
#include<ros/ros.h>
//#include<simple_baxter_gripper_interface/simple_baxter_gripper_interface.h>
#include<generic_gripper_services/genericGripperInterface.h>
//#include<sticky_fingers/StickyControl.h>
#include <std_srvs/SetBool.h>
//BaxterGripper *baxterGripper_ptr;
const double MAX_WAIT_TIME = 3.0; //put a limit on how long will wait for gripper open/close
const double DT=0.01;

ros::ServiceClient *g_client_ptr;

bool callback(generic_gripper_services::genericGripperInterfaceRequest& request,
        generic_gripper_services::genericGripperInterfaceResponse& response) {
        int command_code = request.cmd_code;
        //int ntries = 0;
        double wait_time=0;
        //sticky_fingers::StickyControl sticky_srv;
        std_srvs::SetBool sticky_srv;

        switch(command_code) {
           case generic_gripper_services::genericGripperInterfaceRequest::TEST_PING:
             ROS_INFO("gripper service received a test ping");
             response.success = true; 
             return true;
             break;
             
           case generic_gripper_services::genericGripperInterfaceRequest::RELEASE:  
            ROS_INFO("turning off vacuum--part release");
            sticky_srv.request.data = false;
            if (!g_client_ptr->call(sticky_srv))
            //if (sticky_srv.response.new_status!=false) 
            {
                ROS_WARN("unsuccessful sticky-fingers service call");
            }
            ros::spinOnce();
            //ros::Duration(0.5).sleep();
    
              response.success = true; 
              return true;               
           case generic_gripper_services::genericGripperInterfaceRequest::GRASP_W_PARAMS:
             ROS_WARN("grasp_w_params command not implemented");
             response.success = false; 
             return true;             
           
           case generic_gripper_services::genericGripperInterfaceRequest::GRASP: 
               ROS_INFO("enabling vacuum to grasp part");
               sticky_srv.request.data = true;
               if (!g_client_ptr->call(sticky_srv))
               //if (sticky_srv.response.new_status!=true) 
               {
                ROS_WARN("unsuccessful service call to sticky fingers");
            }
             //  ros::Duration(1.0).sleep();
                response.success = true; 
             return true;             
           case generic_gripper_services::genericGripperInterfaceRequest::TEST_GRASP: 
             ROS_WARN("grasp test not implemented");
             //should look at provided params and decide if gripper pos is between these bounds
             // params are object-dependent; should be obtained from manipulation_properties service
             response.success = false; 
             return true;             
           default:
             ROS_WARN("gripper command not recognized");
             response.success = false; 
             return true;
           }            
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "generic_gripper_interface_svc");
    ros::NodeHandle n;
    //BaxterGripper baxterGripper(&n);
    //baxterGripper_ptr = &baxterGripper; // make this class accessible to callback   
    ros::ServiceServer service = n.advertiseService("generic_gripper_svc", callback);
    //ros::ServiceClient client = n.serviceClient<sticky_fingers::StickyControl>("/sticky_finger/wrist_3_link");
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/sticky_finger/wrist_3_link");

    g_client_ptr = &client; // make this global, so callback can use it
  
    
    ROS_INFO("generic gripper service ready; this version customized for virtual vacuum gripper");
    ros::spin();

    return 0;
}    
