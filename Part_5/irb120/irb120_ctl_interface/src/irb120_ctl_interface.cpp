// irb120_ctl_interface.cpp implementation  file //
// wsn; Dec, 2017
// this node is intended to stand in for "motion_download_interface" of ROS Industrial
// it receives trajectories on topic "joint_path_command"
// it commands each point at specified arrival times; does not interpolate

// rosrun example_robot_interface abb_irb120_interface

#include "irb120_ctl_interface.h"

//class implementation for Irb120RobotInterface
Irb120RobotInterface::Irb120RobotInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { // constructor
    ROS_INFO("in class constructor of InteractivePathMaker");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();

    //initialize variables here, as needed
    ith_point_ = 0;
    npts_traj_ = 0;
    dt_move_ = ros::Duration(0.1);  //publish commands at this period, by default
                                   //within trajectory, each point is published at specified arrival times
    prev_t_from_start_ = ros::Duration(0.0);
    current_t_from_start_ = ros::Duration(0.0);
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass

void Irb120RobotInterface::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");

    sub_joint_trajectory_ = nh_.subscribe("joint_path_command", 0, &Irb120RobotInterface::jointTrajectoryCB, this);
    sub_joint_state_ = nh_.subscribe("/irb120/joint_states", 0, &Irb120RobotInterface::jointStateCB, this);
    // add more subscribers here, as needed
}


//member helper function to set up publishers;

void Irb120RobotInterface::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    joint_command_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectoryPoint>("joint_point_command", 1, true);
    joint1_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint1_position_controller/command", 1, true);
    joint2_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint2_position_controller/command", 1, true);
    joint3_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint3_position_controller/command", 1, true);
    joint4_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint4_position_controller/command", 1, true);
    joint5_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint5_position_controller/command", 1, true);
    joint6_command_publisher_ = nh_.advertise<std_msgs::Float64>("/irb120/joint6_position_controller/command", 1, true);
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



void Irb120RobotInterface::jointTrajectoryCB(const trajectory_msgs::JointTrajectory &traj) {
    //let's copy the message contents:

    njoints_ = traj.joint_names.size();
    npts_traj_=0;// try to avoid race condition
    int npts_traj = traj.points.size();
    ROS_INFO("received trajectory message with %d points", npts_traj);

    new_trajectory_.points.clear();
    new_trajectory_.joint_names.clear();
    for (int i = 0; i < npts_traj; i++) {
        new_trajectory_.points.push_back(traj.points[i]);
        //print_point(traj.points[i]);
    }
    new_trajectory_.header = traj.header;
    new_trajectory_.joint_names = traj.joint_names;
    //print_joint_names(new_trajectory_);

    //new_trajectory_ = msg;
    ith_point_ = 0; // reset the point index--this tells us we have a new trajectory
    npts_traj_ = npts_traj; //last step to avoid race condition
}

void Irb120RobotInterface::jointStateCB(const sensor_msgs::JointState & msg) {
  new_joint_state_ = msg;
}



void Irb120RobotInterface::sendTrajPointCmd() {
    if (ith_point_ < npts_traj_) {
        current_trajectory_point_ = new_trajectory_.points[ith_point_];
        joint_command_publisher_.publish(current_trajectory_point_);
        ith_point_++; // increment to the next point in the trajectory
        
        prev_t_from_start_ = current_t_from_start_;
        current_t_from_start_ = current_trajectory_point_.time_from_start;
        dt_move_ = current_t_from_start_ -prev_t_from_start_;
        pos_cmd_.data = current_trajectory_point_.positions[0];
        joint1_command_publisher_.publish(pos_cmd_);
        pos_cmd_.data = current_trajectory_point_.positions[1];
        joint2_command_publisher_.publish(pos_cmd_);
        pos_cmd_.data = current_trajectory_point_.positions[2];
        joint3_command_publisher_.publish(pos_cmd_);
        pos_cmd_.data = current_trajectory_point_.positions[3];
        joint4_command_publisher_.publish(pos_cmd_);
        pos_cmd_.data = current_trajectory_point_.positions[4];
        joint5_command_publisher_.publish(pos_cmd_);
        pos_cmd_.data = current_trajectory_point_.positions[5];
        joint6_command_publisher_.publish(pos_cmd_);        
        //print_point(current_trajectory_point_);

    } else {
        npts_traj_ = 0;
        ith_point_ = 0;
        dt_move_ = ros::Duration(0.1);
        current_t_from_start_ = ros::Duration(0.0);
    }
}

void Irb120RobotInterface::print_point(trajectory_msgs::JointTrajectoryPoint traj_point) {
    int njoints = traj_point.positions.size();
    ROS_INFO("print_point: njoints = %d", njoints);
    for (int i = 0; i < njoints; i++) {
        ROS_INFO("     jnt %d: %f", i, traj_point.positions[i]);
    }
    ROS_INFO("arrival time: %f",traj_point.time_from_start.toSec());
}

void Irb120RobotInterface::print_joint_names(trajectory_msgs::JointTrajectory traj) {
    int njoints = traj.joint_names.size();
    ROS_INFO("joint names: ");
    for (int i = 0; i < njoints; i++) {
        std::cout << traj.joint_names[i] << ", ";
    }
    std::cout << std::endl;

}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "robotMotionInterface"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands

    ROS_INFO("main: instantiating an object of type Irb120RobotInterface");
    Irb120RobotInterface robotMotionInterface(&nh); //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("going into main loop");
    while (ros::ok()) {
        robotMotionInterface.sendTrajPointCmd();
        ros::spinOnce();
        robotMotionInterface.dt_move_.sleep();
        //sleep_timer.sleep();
    }

    return 0;
}

