//ps7 soln:
//starting_pen_pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service
// this version is useful for having mobot exit the starting pen

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

double g_des_state_x=0.0;
double g_des_state_y=0.0;
double g_des_state_phi=0.0;

void desStateCallback(const nav_msgs::Odometry& des_state_rcvd) {
    // copy some of the components of the received message into member vars
    g_des_state_x = des_state_rcvd.pose.pose.position.x;
    g_des_state_y = des_state_rcvd.pose.pose.position.y;
    geometry_msgs::Quaternion quaternion = des_state_rcvd.pose.pose.orientation;
    g_des_state_phi = convertPlanarQuat2Phi(quaternion); // cheap conversion from quaternion to heading for planar motion    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");

    geometry_msgs::Quaternion quat;
    ros::Subscriber des_state_subscriber= n.subscribe("/desState", 1, desStateCallback);

    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(-M_PI/2.0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = -0.2; //-30.0;
    pose.position.x = -0.2;  
    
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    
    pose.position.y = -0.4; //-30.0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    // let heading settle:
        client.call(path_srv); //
        ros::Duration(2.0).sleep();
        
    path_srv.request.path.poses.clear();

    pose.position.y = -30.0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    
    client.call(path_srv); //
    //monitor progress:
    while (fabs(g_des_state_y+30.0)>0.01) {
        ROS_INFO("des y cmd: %f",g_des_state_y);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    
    ROS_WARN("do manipulation here...");
    ros::Duration(2.0).sleep();
    ROS_INFO("head home: ");
    
    path_srv.request.path.poses.clear();
    pose.position.y = -28.8;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);    
        client.call(path_srv); //

    //let settle:
        ROS_WARN("turn and let settle");
        ros::Duration(2.0).sleep();
    pose.position.y = 0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.clear();

    path_srv.request.path.poses.push_back(pose_stamped);      

    client.call(path_srv); //
    ROS_WARN("head home!");
    while (fabs(g_des_state_y)>0.01) {
        ROS_INFO("des y cmd: %f",g_des_state_y);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    ROS_INFO("done!!");
    return 0;
}
