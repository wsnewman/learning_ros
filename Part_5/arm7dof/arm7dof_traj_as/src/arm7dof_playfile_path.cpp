// arm7dof_playfile_path.cpp:
// example of how to read joint-space path files (w/o timing), bundle them as trajectory goals, and
// send them to the trajectory-interpolation action server

//format of path files:
// entries 0-6 correspond to arm joints 1-7
//each line must contain all 7 values (in fixed order), separated by commas

//the file is read, checked for size consistency (though not for joint-range viability)
// file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.

#include<ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include<arm7dof_traj_as/trajAction.h>

#define VECTOR_DIM 7 // e.g., a 7-dof vector

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

    //line by line, find number of records; they all need to be exactly 7
    //(for path, not trajectory, since no timing information w/ path)
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
    if (max_record_size > 7) {
        ROS_ERROR("bad input file");
        ROS_INFO_STREAM("The largest record has " << max_record_size << " fields.");
        return false;
    }
    if (min_record_size < 7) {
        ROS_ERROR("bad input file");
        ROS_INFO_STREAM("The smallest record has " << min_record_size << " fields.");
        return false;
    }
    //if here, then data has been successfully read and parsed
    return true;
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
bool g_traj_is_done = true;
void armDoneCb(const actionlib::SimpleClientGoalState& state,
        const arm7dof_traj_as::trajResultConstPtr & result) {

    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_traj_is_done=true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm7dof_playfile_path"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec;
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectories  
    bool finished_before_timeout;

    data_t path_data; //read the data files into here

    //open, parse and check the trajectory files
    if (argc == 2) {
        //open the  trajectory file:
        ifstream infile_path(argv[1]);
        if (!infile_path) // file couldn't be opened
        {
            ROS_ERROR("Error: path file could not be opened; halting");
            exit(1);
        }
        if (!read_and_check_file(infile_path, path_data)) {
            exit(1);
        }

        infile_path.close();
    } else {
        ROS_ERROR("must have a path file name as command line argument");
        exit(1);
    }

    //here are hard-coded joint angles for left and right arm poses
    //ROS_INFO("setting pre-poses: ");
    //q_pre_pose_right.resize(7);
    //q_pre_pose_left.resize(7);

    ROS_INFO("instantiating a traj streamer");

    Arm7dof_traj_streamer arm7dof_traj_streamer(&nh); //instantiate a Arm7dof_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    ROS_INFO("warming up callbacks...");
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

    //get current pose of  arm:
    q_vec = arm7dof_traj_streamer.get_q_vec_Xd();

    ROS_INFO_STREAM("arm current state:" << q_vec.transpose());

    //make this the first point of the path, to avoid sudden jumps:
    //std::vector<Eigen::VectorXd> des_path;
    //create a path from the data:
    int npts = path_data.size();
    cout<<"path file has "<<npts<<" points"<<endl;
    des_path.push_back(q_vec);
    for (unsigned n = 0; n < npts; n++) {
        // pack trajectory points, one at a time:
        for (int i = 0; i < 7; i++) {
            q_vec[i] = path_data[n][i];
        }
        cout<<"path pt "<<n<<": "<<q_vec.transpose()<<endl;
        des_path.push_back(q_vec);
    }
    //convert the path to a trajectory:
    cout<<"converting path to trajectory"<<endl;
    //void Arm7dof_traj_streamer::stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {

    arm7dof_traj_streamer.stuff_trajectory(des_path, des_trajectory);

    cout<<"num pts in traj: "<<des_trajectory.points.size()<<endl;
    arm7dof_traj_as::trajGoal goal;
    goal.trajectory = des_trajectory;



    //instantiate client of the arm server:
    actionlib::SimpleActionClient<arm7dof_traj_as::trajAction> arm_action_client("trajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for arm traj action server: ");
    bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm trajectory action server"); // if here, then we connected to the server;  

    //send out this trajectory goal to the trajectory action server
    ROS_INFO("sending goal to arm: ");
    g_traj_is_done=false;
    arm_action_client.sendGoal(goal,&armDoneCb);
    while (!g_traj_is_done) {
        ros::spinOnce();
        ROS_INFO("waiting for traj completion...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("trajectory is done; quitting");

    return 0;
}



