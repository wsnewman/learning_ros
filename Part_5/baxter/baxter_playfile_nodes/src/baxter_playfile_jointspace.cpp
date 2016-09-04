// example of how to read a joint-space trajectory stored line-by-line in CSV file format, convert to trajectory and play it
//specialized for Baxter arms, fixed order of joints
// entries 0-6 correspond to right arm, joints 1-7
//each line must contain all 7 values (in fixed order), separated by commas

//the file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
// file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.

// run this as: rosrun baxter_playfile_nodes baxter_playfile_jointspace fname_right.jsp fname_left.jsp
// where fname_right.jsp and fname_left.jsp are desired right and left-arm playfile names
// optionally, just give a single playfile name, which will be interpreted as the right-arm file

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include <std_msgs/UInt32.h>
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

bool g_got_code_trigger = false;
int g_playfile_code = 0;
bool g_got_good_traj_right = false;
bool g_got_good_traj_left = false;
bool g_right_arm_done = false;
bool g_left_arm_done = false;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr & result) {

    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_right_arm_done = true;
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr & result) {

    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_left_arm_done = true;
}

void playfileCB(const std_msgs::UInt32& code_msg) {
    g_playfile_code = code_msg.data;
    ROS_INFO("received code: %d", g_playfile_code);
    g_got_code_trigger = true;
}


//open a named file and read it into a joint trajectory message:

int read_traj_file(string fname, trajectory_msgs::JointTrajectory &des_trajectory) {
    //open the trajectory file:
    ifstream infile(fname.c_str());
    if (infile.is_open()) {
        ROS_INFO("opened file");
    }
    if (!infile) // file couldn't be opened
    {
        cerr << "Error: file " << fname << " could not be opened" << endl;
        return 1;
    }
    cout << "opened file " << fname << endl;

    // define a vector of desired joint displacements...w/o linkage redundancies
    //7'th angle is related to jaw opening--but not well handled yet
    //Vectorq7x1 q_vec,q_vec2;
    //vector <Vectorq7x1> q1_vecs,q2_vecs;
    //vector <double> arrival_times;

    // Here is the data we want.
    data_t data;

    // Here is the file containing the data. Read it into data.
    infile >> data;

    // Complain if something went wrong.
    if (!infile.eof()) {
        cout << "error reading file!\n";
        return 1;
    }

    infile.close();


    // Otherwise, list some basic information about the file.
    cout << "CSV file contains " << data.size() << " records.\n";

    unsigned min_record_size = data[0].size();
    unsigned max_record_size = 0;
    for (unsigned n = 0; n < data.size(); n++) {
        if (max_record_size < data[ n ].size())
            max_record_size = data[ n ].size();
        if (min_record_size > data[ n ].size())
            min_record_size = data[ n ].size();
    }
    if (max_record_size > 8) {
        ROS_WARN("bad file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

    }
    if (min_record_size < 8) {
        ROS_WARN("bad file");
        cout << "The smallest record has " << min_record_size << " fields.\n";
        return 1;

    }
    //cout << "The second field in the fourth record contains the value " << data[ 3 ][ 1 ] << ".\n";

    //data is valid; pack it up as a trajectory and ship it

    //trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
    // if using wsn's trajectory streamer action server
    des_trajectory.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint trajectory_point; //,trajectory_point2; 
    trajectory_point.positions.resize(7);
    double t_arrival;
    for (unsigned n = 0; n < data.size(); n++) {
        // pack trajectory points, one at a time:
        for (int i = 0; i < 7; i++) {
            trajectory_point.positions[i] = data[n][i];
        }
        t_arrival = data[n][7];
        trajectory_point.time_from_start = ros::Duration(t_arrival);
        des_trajectory.points.push_back(trajectory_point);
    }
    return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    Eigen::VectorXd q_right_state, q_right_firstpoint, q_left_state, q_left_firstpoint;
    q_right_firstpoint.resize(7);
    q_left_firstpoint.resize(7);
    Eigen::VectorXd dqvec;
    dqvec.resize(7);
    Vectorq7x1 q_vec_right_arm, q_vec_left_arm;

    std::vector<Eigen::VectorXd> des_path_right, des_path_left;

    trajectory_msgs::JointTrajectory des_trajectory_right, des_trajectory_left; // objects to hold trajectories
    trajectory_msgs::JointTrajectory approach_trajectory_right, approach_trajectory_left; // objects to hold trajectories    
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;

    if (argc < 2) {
        ROS_WARN("usage error: no filename listed");
        ROS_WARN("use: rosrun baxter_playfile_nodes baxter_playback rt_fname.jsp [lft_fname.jsp]");
        ROS_WARN("make sure the right (and optional left) playfiles are in the current directory");
        exit(1);
    }
    int num_arms_ctl = 0; //will get set to 1 (rt arm only) or 2 (both arms)
    if (argc == 2) {
        num_arms_ctl = 1;
    }
    if (argc == 3) {
        num_arms_ctl = 2;
    }
    //cout << "instantiating a traj streamer" << endl;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    //cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

    baxter_trajectory_streamer::trajGoal goal_right, goal_left;

    //instantiate clients of the two arm servers:
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client("leftArmTrajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

    if (num_arms_ctl == 2) {
        ROS_INFO("waiting for left-arm server: ");
        server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
        while (!server_exists) {
            ROS_WARN("waiting on left-arm server...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
        }
        ROS_INFO("connected to left-arm action server"); // if here, then we connected to the server; 
    }


    //here's the main loop:
    ros::spinOnce();

    //get right and left arm angles; start motion from here
    q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
    q_right_state = q_vec_right_arm; // start from here;
    ROS_INFO_STREAM("right-arm current state:" << q_vec_right_arm.transpose());

    q_vec_left_arm = baxter_traj_streamer.get_qvec_left_arm();
    q_left_state = q_vec_left_arm; // start from here;  
    ROS_INFO_STREAM("left-arm current state:" << q_vec_left_arm.transpose());

    des_path_right.clear();
    des_path_right.push_back(q_right_state);
    des_path_left.clear();
    des_path_left.push_back(q_left_state);

    g_got_good_traj_right = false;

    //open, parse and check the trajectory files
    //open the first (right-arm) trajectory file:
    if (0 == read_traj_file(argv[1], des_trajectory_right)) {
        ROS_INFO("read file OK");
        g_got_good_traj_right = true;
    } else {
        ROS_ERROR("could not read right-arm file");
        exit(1);
    }

    g_got_good_traj_left = false;
    if (num_arms_ctl == 2) {
        if (0 == read_traj_file(argv[2], des_trajectory_left)) {
            ROS_INFO("read left-arm file OK");
            g_got_good_traj_left = true;
        } else {
            ROS_ERROR("could not read right-arm file");
            exit(1);
        }
    }

    //now have current arm poses and desired trajectories; splice in a motion from current arm pose
    // to first point of desired trajectory;
    if (g_got_good_traj_right) {
        //get first pt of traj: q_right_firstpoint
        trajectory_point0 = des_trajectory_right.points[0];
        for (int i = 0; i < 7; i++) { //copy from traj point to Eigen-type vector
            q_right_firstpoint[i] = trajectory_point0.positions[i];
        }
        //add this pt to path from current pose:
        des_path_right.push_back(q_right_firstpoint);
        //use traj stuffer to find build trajectory from current pose to first point of recorded traj
        baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, approach_trajectory_right);
    }
    if (g_got_good_traj_left) {
        trajectory_point0 = des_trajectory_left.points[0];
        for (int i = 0; i < 7; i++) { //copy from traj point to Eigen-type vector
            q_left_firstpoint[i] = trajectory_point0.positions[i];
        }
        //add this pt to path from current pose:
        des_path_left.push_back(q_left_firstpoint);
        //use traj stuffer to find build trajectory from current pose to first point of recorded traj
        baxter_traj_streamer.stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
    }

    //command the approach trajectory/trajectories and wait for conclusion   
    g_right_arm_done = true;

    if (g_got_good_traj_right) {
        goal_right.trajectory = approach_trajectory_right;
        g_right_arm_done = false; //reset status trigger, so can check when done
        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); // we could also name additional callback functions here, if desired
        //    right_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    }
    g_left_arm_done = true;
    if (g_got_good_traj_left) {
        goal_left.trajectory = approach_trajectory_left;
        g_left_arm_done = false;
        left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
    }
    while (!g_right_arm_done || !g_left_arm_done) {
        ROS_INFO("waiting on arm server(s) to approach start of traj");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    // now send the desired trajectory from file, if there are any points left to execute
    if (g_got_good_traj_right && (des_trajectory_right.points.size() > 1)) {
        goal_right.trajectory = des_trajectory_right;
        g_right_arm_done = false; //reset status trigger, so can check when done
        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); // we could also name additional callback functions here, if desired
        //    right_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    }
    if (g_got_good_traj_left && (des_trajectory_left.points.size() > 1)) {
        goal_left.trajectory = des_trajectory_left;
        g_left_arm_done = false; //reset status trigger, so can check when done
        left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
    }

    while (!g_right_arm_done || !g_left_arm_done) {
        ROS_INFO("waiting on arm server(s) to execute playfile(s)");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}




