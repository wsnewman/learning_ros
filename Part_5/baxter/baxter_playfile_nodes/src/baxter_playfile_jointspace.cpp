// example of how to read joint-space playfiles, bundle them as trajectory goals, and
// send them to the left and right-arm trajectory-interpolation action servers

//format of the trajectory files:
// entries 0-6 correspond to arm joints 1-7; 8th entr is arrival time (in seconds)
//each line must contain all 7 values (in fixed order), separated by commas

//the file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
// file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.
// performed separately for right and left arms.

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include<baxter_trajectory_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator >>(istream& ins, record_t& record) {
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline(ins, line);

    // now we'll use a stringstream to separate the fields out of the line
    stringstream ss(line);
    string field;
    while (getline(ss, field, ',')) {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs(field);
        double f = 0.0; // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        record.push_back(f);
    }

    // Now we have read a single line, converted into a list of fields, converted the fields
    // from strings to doubles, and stored the results in the argument record, so
    // we just return the argument stream as required for this kind of input overload function.
    return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator >>(istream& ins, data_t& data) {
    // make sure that the returned data only contains the CSV data we read here
    data.clear();

    // For every record we can read from the file, append it to our resulting data
    record_t record;
    while (ins >> record) {
        data.push_back(record);
    }

    // Again, return the argument stream as required for this kind of input stream overload.
    return ins;
}

// fnc to open named file, read the data, and check that all rows have the required number of entries

bool read_and_check_file(ifstream &infile, data_t &data) {
    infile >> data;
    // Complain if something went wrong.
    if (!infile.eof()) {
        ROS_ERROR("error reading file !");
        return false;
    }
    //parse for file integrity:
    // Otherwise, list some basic information about the file.
    ROS_INFO_STREAM("CSV file contains" << data.size() << " records");

    //line by line, find number of records; they all need to be exactly 8
    unsigned min_record_size = data[0].size();
    unsigned max_record_size = 0;
    for (unsigned n = 0; n < data.size(); n++) {
        if (max_record_size < data[ n ].size()) {
            max_record_size = data[ n ].size();
        }
        if (min_record_size > data[ n ].size()) {
            min_record_size = data[ n ].size();
        }
    }
    if (max_record_size > 8) {
        ROS_ERROR("bad input file");
        ROS_INFO_STREAM("The largest record has " << max_record_size << " fields.");
        return false;
    }
    if (min_record_size < 8) {
        ROS_ERROR("bad input file");
        ROS_INFO_STREAM("The smallest record has " << min_record_size << " fields.");
        return false;
    }
    //if here, then data has been successfully read and parsed
    return true;
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr & result) {

    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr & result) {

    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    Eigen::VectorXd q_pre_pose_right, q_pre_pose_left;
    Eigen::VectorXd q_vec_right_arm, q_vec_left_arm;
    std::vector<Eigen::VectorXd> des_path_right, des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left; // empty trajectories  

    data_t data_right, data_left; //read the data files into here
    int num_arms_ctl = 0; //will get set to 1 (rt arm only) or 2 (both arms)

    //open, parse and check the trajectory files
    if (argc == 2) {
        num_arms_ctl = 1;

        //open the first (right-arm) trajectory file:
        ifstream infile_right(argv[1]);
        if (!infile_right) // file couldn't be opened
        {
            ROS_ERROR("Error: right-arm file could not be opened; halting");
            exit(1);
        }
        if (!read_and_check_file(infile_right, data_right)) {
            exit(1);
        }

        infile_right.close();
    } else if (argc == 3) {
        num_arms_ctl = 2; //control both arms
        //open two trajectory files:
        //open the first (right-arm) trajectory file:
        ifstream infile_right(argv[1]);
        if (!infile_right) // file couldn't be opened
        {
            ROS_ERROR("Error: right-arm file could not be opened; halting");
            exit(1);
        }
        if (!read_and_check_file(infile_right, data_right)) {
            exit(1);
        }

        infile_right.close();

        //open the second (left-arm) trajectory file:
        ifstream infile_left(argv[2]);
        if (!infile_left) // file couldn't be opened
        {
            ROS_ERROR("Error: left-arm file could not be opened; halting");
            exit(1);
        }
        if (!read_and_check_file(infile_left, data_left)) {
            exit(1);
        }

        infile_left.close();
    } else {
        ROS_ERROR("must have one or two file names as command line arguments");
        exit(1);
    }

       //here are hard-coded joint angles for left and right arm poses
    ROS_INFO("setting pre-poses: ");
    q_pre_pose_right.resize(7);
    q_pre_pose_left.resize(7);

    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    ROS_INFO("warming up callbacks...");
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

    //get current pose of left and right arms:
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();
    ROS_INFO_STREAM("right-arm current state:" << q_vec_right_arm.transpose());

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd();
    ROS_INFO_STREAM("left-arm current state:" << q_vec_left_arm.transpose());

    //des_path_right.push_back(q_vec_right_arm); //should start from current pose
    //des_path_left.push_back(q_vec_left_arm);
    
    //data is valid; pack it up as trajectories and ship them out
    des_trajectory_right.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory_right.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
    // if using wsn's trajectory streamer action server
    des_trajectory_right.header.stamp = ros::Time::now();
    //same for left-arm trajectory:
    des_trajectory_left.points.clear();
    des_trajectory_left.joint_names.clear();
    des_trajectory_left.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(7);
    double t_arrival;
    for (unsigned n = 0; n < data_right.size(); n++) {
        // pack trajectory points, one at a time:
        for (int i = 0; i < 7; i++) {
            trajectory_point.positions[i] = data_right[n][i];
        }
        t_arrival = data_right[n][7];
        trajectory_point.time_from_start = ros::Duration(t_arrival);
        des_trajectory_right.points.push_back(trajectory_point);
    }
    //repeat for left arm:
    if (num_arms_ctl == 2) {
        for (unsigned n = 0; n < data_left.size(); n++) {
            // pack trajectory points, one at a time:
            for (int i = 0; i < 7; i++) {
                trajectory_point.positions[i] = data_left[n][i];
            }
            t_arrival = data_left[n][7];
            trajectory_point.time_from_start = ros::Duration(t_arrival);
            des_trajectory_left.points.push_back(trajectory_point);
        }
    }

    //now have the data in a trajectory; send it to the trajectory streamer action server to execute
    // goal objects compatible with the arm servers
    baxter_trajectory_streamer::trajGoal goal_right, goal_left;
    //  copy traj to goal:
    goal_right.trajectory = des_trajectory_right;
    goal_left.trajectory = des_trajectory_left;

    //instantiate clients of the two arm servers:
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client("leftArmTrajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

    if (num_arms_ctl == 2) {
        ROS_INFO("waiting for left-arm server: ");
        server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
        while (!server_exists) {
            ROS_WARN("waiting on left-arm server...");
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
        }
        ROS_INFO("connected to left-arm action server"); // if here, then we connected to the server; 
    }
    if (num_arms_ctl == 1) {
        ROS_INFO("sending goal to right arm: ");
        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

    } else {
        ROS_INFO("sending goals to left and right arms: ");
        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);
        left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb);
    }
    return 0;
}



