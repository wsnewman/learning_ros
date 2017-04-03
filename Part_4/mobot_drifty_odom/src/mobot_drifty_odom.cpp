#include <ros/ros.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <tf/transform_broadcaster.h>

//from URDF:  <xacro:property name="tirediam" value="0.3302" />
const double R_LEFT_WHEEL = 0.3302 / 2.0;
const double R_RIGHT_WHEEL = R_LEFT_WHEEL + 0.005; //introduce error--tire diam diff
//from URDF: <xacro:property name="track" value=".56515" />
const double TRACK = 0.560515; //0.56515; //0.560; // track error

const double wheel_ang_sham_init = -1000000.0;
bool joints_states_good = false;

nav_msgs::Odometry g_drifty_odom;
sensor_msgs::JointState g_joint_state;
ros::Publisher g_drifty_odom_pub;
ros::Subscriber g_joint_state_subscriber;
//tf::TransformBroadcaster* g_odom_broadcaster_ptr;
geometry_msgs::TransformStamped g_odom_trans;

double g_new_left_wheel_ang, g_old_left_wheel_ang;
double g_new_right_wheel_ang, g_old_right_wheel_ang;
double g_t_new, g_t_old, g_dt;
ros::Time g_cur_time;
double g_odom_psi;

geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

void joint_state_CB(const sensor_msgs::JointState& joint_states) {
    double dtheta_right, dtheta_left, ds, dpsi;
    int n_joints = joint_states.name.size();
    int ijnt;
    int njnts_found = 0;
    bool found_name = false;

    g_old_left_wheel_ang = g_new_left_wheel_ang;
    g_old_right_wheel_ang = g_new_right_wheel_ang;
    g_t_old = g_t_new;
    g_cur_time = ros::Time::now();
    g_t_new = g_cur_time.toSec();
    g_dt = g_t_new - g_t_old;

    for (ijnt = 0; ijnt < n_joints; ijnt++) {
        std::string joint_name(joint_states.name[ijnt]);
        if (joint_name.compare("left_wheel_joint") == 0) {
            g_new_left_wheel_ang = joint_states.position[ijnt];
            njnts_found++;
        }
        if (joint_name.compare("right_wheel_joint") == 0) {
            g_new_right_wheel_ang = joint_states.position[ijnt];
            njnts_found++;
        }
    }
    if (njnts_found < 2) {
         //ROS_WARN("did not find both wheel joint angles!");
         //for (ijnt = 0; ijnt < n_joints; ijnt++) {
         //    std::cout<<joint_states.name[ijnt]<<std::endl;
         //}
    }
    else {
        //ROS_INFO("found both wheel joint names");
    }
    if (!joints_states_good) {
        if (g_new_left_wheel_ang > wheel_ang_sham_init / 2.0) {
            joints_states_good = true; //passed the test
            g_old_left_wheel_ang = g_new_left_wheel_ang;
            g_old_right_wheel_ang = g_new_right_wheel_ang; //assume right is good now as well          
        }
    }
    if (joints_states_good) { //only compute odom if wheel angles are valid
        dtheta_left = g_new_left_wheel_ang - g_old_left_wheel_ang;
        dtheta_right = g_new_right_wheel_ang - g_old_right_wheel_ang;
        ds = 0.5 * (dtheta_left * R_LEFT_WHEEL + dtheta_right * R_RIGHT_WHEEL);
        dpsi = dtheta_right * R_RIGHT_WHEEL / TRACK - dtheta_left * R_LEFT_WHEEL / TRACK;

        g_drifty_odom.pose.pose.position.x += ds * cos(g_odom_psi);
        g_drifty_odom.pose.pose.position.y += ds * sin(g_odom_psi);
        g_odom_psi += dpsi;
        //ROS_INFO("dthetal, dthetar, dpsi, odom_psi, dx, dy= %f, %f %f, %f %f %f", dtheta_left, dtheta_right, dpsi, g_odom_psi,
        //        ds * cos(g_odom_psi), ds * sin(g_odom_psi));
        g_drifty_odom.pose.pose.orientation = convertPlanarPsi2Quaternion(g_odom_psi);

        if (g_dt>0.0005) {
          g_drifty_odom.twist.twist.linear.x = ds / g_dt;
          g_drifty_odom.twist.twist.angular.z = dpsi / g_dt;
        } 
        g_drifty_odom.header.stamp = g_cur_time;
        g_drifty_odom_pub.publish(g_drifty_odom);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drifty_odom_publisher");
    ros::NodeHandle nh; 
    //inits:
    g_new_left_wheel_ang = wheel_ang_sham_init;
    g_old_left_wheel_ang = wheel_ang_sham_init;
    g_new_right_wheel_ang = wheel_ang_sham_init;
    g_old_right_wheel_ang = wheel_ang_sham_init;
    g_cur_time = ros::Time::now();
    g_t_new = g_cur_time.toSec();
    g_t_old = g_t_new;

    //initialize odom with pose and twist defined as zero at start-up location
    g_drifty_odom.child_frame_id = "base_link";
    g_drifty_odom.header.frame_id = "drifty_odom";
    g_drifty_odom.header.stamp = g_cur_time;
    g_drifty_odom.pose.pose.position.x = 0.0;
    g_drifty_odom.pose.pose.position.y = 0.0;
    g_drifty_odom.pose.pose.position.z = 0.0;
    g_drifty_odom.pose.pose.orientation.x = 0.0;
    g_drifty_odom.pose.pose.orientation.y = 0.0;
    g_drifty_odom.pose.pose.orientation.z = 0.0;
    g_drifty_odom.pose.pose.orientation.w = 1.0;

    g_drifty_odom.twist.twist.linear.x = 0.0;
    g_drifty_odom.twist.twist.linear.y = 0.0;
    g_drifty_odom.twist.twist.linear.z = 0.0;
    g_drifty_odom.twist.twist.angular.x = 0.0;
    g_drifty_odom.twist.twist.angular.y = 0.0;
    g_drifty_odom.twist.twist.angular.z = 0.0;
    
    ros::Rate timer(100.0); // a 100Hz timer

    g_drifty_odom_pub = nh.advertise<nav_msgs::Odometry>("drifty_odom", 1);
    g_joint_state_subscriber = nh.subscribe("joint_states", 1, joint_state_CB);
    while (ros::ok()) {
        ros::spin();
    }
}
