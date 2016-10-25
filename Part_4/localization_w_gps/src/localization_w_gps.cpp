// localization_w_gps.cpp//
// wsn; Oct, 2016
// integrates noisy GPS, IMU and flawed odometry to estimate pose in world coords


//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <xform_utils/xform_utils.h>

const double MAIN_DT=0.01; //100Hz
const double K_YAW = 0.1; //gain for heading corrections
const double K_GPS= 0.002; // gain for GPS corrections
const double L_MOVE = 0.1; //motion to travel, in m, before heading updates from GPS

XformUtils xform_utils; // for type conversions

//globals for communication w/ callbacks:
double g_omega_z_imu=0; //imu
bool g_imu_good = false;
sensor_msgs::Imu g_imu;

double g_x_gps=0; //gps
double g_y_gps=0;
bool g_gps_good=false;
geometry_msgs::Pose g_gps_pose;

bool g_odom_good=false; //odom
double g_odom_speed=0;
nav_msgs::Odometry g_current_odom;

//yaw from ideal gps/gazebo model state
double g_true_yaw=0; //actual heading, as reported by gazebo w/o noise;
   //merely for debug/display purposes--can not use for localization


//receive publications from gazebo via node mobot_gazebo_state;
//contains noisy position estimate in world coords
void gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose) {
   g_x_gps = gazebo_pose.position.x; //copy the state to member variables of this object
   g_y_gps = gazebo_pose.position.y;
   g_gps_pose = gazebo_pose;
   g_gps_good = true;
}

//for debug, get actual heading from noiseless gazebo state
void gazeboTrueYawCallback(const geometry_msgs::Pose& gazebo_pose) {
   geometry_msgs::Quaternion  state_quat = gazebo_pose.orientation;
   g_true_yaw = xform_utils.convertPlanarQuat2Phi(state_quat);
}

//drifty odom info--only care about speed here
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    g_current_odom = odom_rcvd;
    g_odom_speed = odom_rcvd.twist.twist.linear.x;
    g_odom_good=true;   
}

//imu callback: 
void imuCallback(const sensor_msgs::Imu& imu_rcvd) {
    g_imu = imu_rcvd;
    g_omega_z_imu = g_imu.angular_velocity.z;
    g_imu_good = true;
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "gps_localizer"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    geometry_msgs::PoseStamped pose_estimate;
    std_msgs::Float64 yaw_msg;
    geometry_msgs::Quaternion quat_est;
    double dx_odom_est=0;
    double dy_odom_est=0;
    double dl_odom_est=0;
    
    double yaw_est=0;
    double x_est=0;
    double y_est=0;
    double x_est_old = 0;
    double y_est_old = 0;    
    
    ros::Rate timer(1/MAIN_DT);
    ros::Subscriber gps_subscriber = nh.subscribe("gazebo_mobot_noisy_pose", 1, gazeboPoseCallback); 
    //test: can use ideal, noiseless pose:
    //ros::Subscriber gps_subscriber = nh.subscribe("gazebo_mobot_pose", 1, gazeboPoseCallback);     
    ros::Subscriber imu_subscriber = nh.subscribe("/imu_data", 1, imuCallback); 
    ros::Subscriber odom_subscriber = nh.subscribe("/drifty_odom",1,odomCallback);
    ros::Subscriber true_state_subscriber = nh.subscribe("gazebo_mobot_pose",1,gazeboTrueYawCallback);
    ros::Publisher localization_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mobot_localization", 1);
    ros::Publisher yaw_publisher = nh.advertise<std_msgs::Float64>("/yaw_estimate",1);
    ros::Publisher true_yaw_publisher = nh.advertise<std_msgs::Float64>("/true_yaw",1);
    
    ROS_INFO("warm up callbacks: ");
    ROS_INFO("waiting on gps: ");
    while (!g_gps_good) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
            }
    pose_estimate.header.stamp = ros::Time::now();
    pose_estimate.pose.position.x = g_x_gps;
    pose_estimate.pose.position.y = g_y_gps;    
    pose_estimate.pose.position.z = 0;
    
    ROS_INFO("got gps info; wait for imu:");
    while (!g_imu_good) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();        
    }
    ROS_INFO("imu is good; wait for odom:");
    while (!g_odom_good) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();        
    }    
    double dang_gps=0;
    double dang_odom=0;
    double dx_odom,dy_odom;
    double delta_odom_x=0;
    double delta_odom_y=0;
    double yaw_err=0;
    double move_dist = 0;
    //main, timed loop:
    x_est = g_x_gps;
    x_est_old = x_est;
    y_est = g_y_gps;
    y_est_old = y_est;
    while(ros::ok()) {
        ros::spinOnce();

        x_est = (1-K_GPS)*x_est + K_GPS*g_x_gps; //incorporate gps feedback
        y_est = (1-K_GPS)*y_est + K_GPS*g_y_gps; //ditto
        dl_odom_est = MAIN_DT*g_odom_speed; //moved this far in 1 DT
        move_dist+= dl_odom_est; //keep track of cumulative move distance
        yaw_est+= MAIN_DT*g_omega_z_imu; //integrate the IMU's yaw to estimate heading
            if (yaw_est<-M_PI) yaw_est+= 2.0*M_PI; //remap periodically
            if (yaw_est>M_PI) yaw_est-= 2.0*M_PI;        
        
        dx_odom = dl_odom_est*cos(yaw_est); //incremental x and y motions, as
        dy_odom = dl_odom_est*sin(yaw_est); //inferred from speed and heading est
        x_est+= dx_odom; //cumulative x and y estimates updated from odometry
        y_est+= dy_odom;
        delta_odom_x+= dx_odom; //keep track of x, y displacements over specified
        delta_odom_y+= dy_odom; // distances for yaw updates
        if (fabs(move_dist) > L_MOVE) { //if moved this far since last yaw update,
            //time to do another yaw update based on GPS
            //since last update, express motion in polar coords based on gps
            dang_gps = atan2((y_est-y_est_old),(x_est-x_est_old));
            //similarly, update in polar coords based on odometry
            dang_odom = atan2(delta_odom_y,delta_odom_x);
            //if gps and odom disagree on avg heading, make a correction
            yaw_err = dang_gps - dang_odom;
            if (yaw_err>M_PI) yaw_err-=2.0*M_PI;
            if (yaw_err<-M_PI) yaw_err+=2.0*M_PI;
            //K_YAW should not be lareger than unity; smaller--> less noise sensitivity
            yaw_est+=K_YAW*yaw_err; //here's the yaw update due to gps

            y_est_old=y_est; //save this state as a checkpoint for next yaw update
            x_est_old=x_est;
            move_dist=0;
            delta_odom_y=0;
            delta_odom_x=0;
        }
        
        yaw_msg.data = yaw_est; //publish the yaw estimate, for use/display
        yaw_publisher.publish(yaw_msg);
        
        yaw_msg.data =g_true_yaw; //pub actual yaw, for comparison
        true_yaw_publisher.publish(yaw_msg);
        //publish x and y estimates 
        pose_estimate.header.stamp = ros::Time::now();
        pose_estimate.pose.position.x = x_est;
        pose_estimate.pose.position.y = y_est;    
        // convert heading to quaternion and publish here as well
        quat_est = xform_utils.convertPlanarPsi2Quaternion(yaw_est);
        pose_estimate.pose.orientation = quat_est;
        localization_publisher.publish(pose_estimate);

        timer.sleep();
        ros::spinOnce();
    }
  return 0;
}