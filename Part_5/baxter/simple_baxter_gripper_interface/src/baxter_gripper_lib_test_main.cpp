// baxter_gripper_lib_test_main: 
// wsn, August, 2016
// illustrates use of library/class BaxterGripper for simplified Baxter gripper I/O

#include<ros/ros.h>
#include<simple_baxter_gripper_interface/simple_baxter_gripper_interface.h>
//using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_gripper_test_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   

    //instantiate a BaxterGripper object to do gripper I/O
    BaxterGripper baxterGripper(&nh);
    //wait for filter warm-up on right-gripper position
    while (baxterGripper.get_right_gripper_pos()<-0.5) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        ROS_INFO("waiting for right gripper position filter to settle; pos = %f", baxterGripper.get_right_gripper_pos());
    }

    ROS_INFO("closing right gripper");
    baxterGripper.right_gripper_close();
    ros::Duration(1.0).sleep();
    ROS_INFO("opening right gripper");
    baxterGripper.right_gripper_open();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos>95", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() < 95.0) {
        baxterGripper.right_gripper_open();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }

    ROS_INFO("closing left gripper");
    baxterGripper.left_gripper_close();
    ros::Duration(1.0).sleep();
    ROS_INFO("opening left gripper");
    baxterGripper.left_gripper_open();

    ROS_INFO("closing right gripper");
    baxterGripper.right_gripper_close();
    ros::spinOnce();
    ROS_INFO("right gripper pos = %f; waiting for pos<90", baxterGripper.get_right_gripper_pos());
    while (baxterGripper.get_right_gripper_pos() > 90.0) {
        baxterGripper.right_gripper_close();
        ros::spinOnce();
        ROS_INFO("gripper pos = %f", baxterGripper.get_right_gripper_pos());
        ros::Duration(0.01).sleep();
    }

    return 0;
}

