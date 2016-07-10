// example of how to read a joint-space trajectory stored line-by-line in CSV file format, convert to trajectory and play it
//specialized for Baxter right arm, fixed order of joints
// entries 0-6 correspond to right arm, joints 1-7
//each line must contain all 7 values (in fixed order), separated by commas

//the file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
// file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.
// 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <std_msgs/UInt32.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
// Nov 3, 2015 update: moved action message to cwru_action...
// all code using this streamer will need to include cwru_action and use action message here
#include<cwru_action/trajAction.h>
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

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const cwru_action::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

bool g_got_code_trigger = false;
int g_alexa_code = 0;
bool g_got_good_traj = false;

void alexaCB(const std_msgs::UInt32& code_msg) {
    g_alexa_code = code_msg.data;
    ROS_INFO("received code: %d", g_alexa_code);
    g_got_code_trigger = true;
}


//open a named file and read it into a joint trajectory message:

int read_traj_file(string fname, trajectory_msgs::JointTrajectory &des_trajectory) {
    //open the trajectory file:
    ifstream infile(fname.c_str());
    if (infile.is_open()) { ROS_INFO("opened file"); }
    if (!infile) // file couldn't be opened
    {
        cerr << "Error: file "<<fname<<" could not be opened; halting" << endl;
        exit(1);
    }
    cout<<"opened file "<<fname<<endl;

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
    //instantiate a DavinciJointPublisher object and pass in pointer to nodehandle for constructor to use

    ros::Subscriber traj_code = nh.subscribe("/Alexa_codes", 1, alexaCB);
    int g_count = 0;
    int ans;
    Vectorq7x1 q_pre_pose;
    //q_in << 0, 0, 0, 0, 0, 0, 0;  
    q_pre_pose << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;


    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    //cin>>ans;
    // here is a "goal" object compatible with the server, as defined in example_action_server/action    
    // copy traj to goal:   
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    cwru_action::trajGoal goal;

    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    cout << "instantiating a traj streamer" << endl; // enter 1:";
    //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout << "getting current right-arm pose:" << endl;
    q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
    cout << "r_arm state:" << q_vec_right_arm.transpose() << endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here
    q_in_vecxd = q_pre_pose; // conversion; not sure why I needed to do this...but des_path.push_back(q_in_vecxd) likes it
    des_path.push_back(q_in_vecxd); //twice, to define a trajectory  

    /*
    if (argc!=2) {
      ROS_INFO("argc= %d; missing file command-line argument; halting",argc);
    return 0;
    }
     * */

    // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
    actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 


    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
    
    
    //here's the main loop:
    while (ros::ok()) {
        ros::spinOnce();
        if (g_got_code_trigger) {
            g_got_code_trigger = false;
            switch (g_alexa_code) {
                case 1:
                    ROS_INFO("case 1:  merry_r_arm_traj.jsp");
                    if (0 == read_traj_file("merry_r_arm_traj.jsp", des_trajectory)) {
                        ROS_INFO("read file OK");
                        g_got_good_traj = true;}
                    else ROS_ERROR("could not read file");
                    break;
                case 2:
                    ROS_INFO("case 2: ");
                    if (0 == read_traj_file("merry_r_arm_traj2.jsp", des_trajectory))
                    g_got_good_traj = true;
                    break;
                default:
                    ROS_INFO("unknown case");
                    break;
            }

        }


        //now have the data in a trajectory; send it to the trajectory streamer action server to execute

        //  copy traj to goal:
        if (g_got_good_traj) {
            goal.trajectory = des_trajectory;
            //cout<<"ready to connect to action server; enter 1: ";
            //cin>>ans;


            action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
            //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

            bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result for goal number %d", g_count);
            } else {
                ROS_INFO("finished before timeout");
            }
            g_got_good_traj = false;
        }
    }

    return 0;
}




