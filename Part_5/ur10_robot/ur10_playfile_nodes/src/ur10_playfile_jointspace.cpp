// example of how to read a joint-space trajectory stored line-by-line in CSV file format, convert to trajectory and play it
//specialized for UR10 arm, fixed order of joints
// entries 0-5 correspond to joints 1-6
//each line must contain all 6 values (in fixed order), separated by commas

//the file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
// file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory action server.

// run this as: rosrun baxter_playfile_nodes baxter_playfile_jointspace fname_right.jsp fname_left.jsp
// where fname_right.jsp and fname_left.jsp are desired right and left-arm playfile names
// optionally, just give a single playfile name, which will be interpreted as the right-arm file

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <ur_fk_ik/ur_kin.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


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

const double dt_traj = 0.02; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
         
//double g_qdot_max_vec[] = {2.16, 2.16, 3.15, 3.2, 3.2, 3.2}; //values per URDF ur10.urdf.xacro in ur_description

void set_ur_jnt_names() {
    g_ur_jnt_names.push_back("shoulder_pan_joint");
    g_ur_jnt_names.push_back("shoulder_lift_joint");
    g_ur_jnt_names.push_back("elbow_joint");
    g_ur_jnt_names.push_back("wrist_1_joint");
    g_ur_jnt_names.push_back("wrist_2_joint");
    g_ur_jnt_names.push_back("wrist_3_joint");
}

double transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_ur_jnt_names
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
    }

    //new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = 0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    //cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }

}

//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_ur_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        //cout << "CB: q_vec_right_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}


//action server will respond to this callback when done
void armDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    g_done_move = true;
    //ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
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
    if (max_record_size > VECTOR_DIM+1) {
        ROS_WARN("bad file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

    }
    if (min_record_size < VECTOR_DIM+1) {
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
    trajectory_point.positions.resize(VECTOR_DIM);
    double t_arrival;
    for (unsigned n = 0; n < data.size(); n++) {
        // pack trajectory points, one at a time:
        for (int i = 0; i < VECTOR_DIM; i++) {
            trajectory_point.positions[i] = data[n][i];
        }
        t_arrival = data[n][VECTOR_DIM];
        trajectory_point.time_from_start = ros::Duration(t_arrival);
        des_trajectory.points.push_back(trajectory_point);
    }
    return 0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

     Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    set_ur_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //here are initial, hard-coded joint angles for arm pose
    cout << "setting pre-pose: " << endl;
    q_pre_pose.resize(VECTOR_DIM);
    q_pre_pose << 0, 0, 0, 0, 0, 0; //-0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //get current pose of arm:  
    cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;

    //instantiate client of the arm server:
    //create an action client of UR's follow_joint_trajectory action server 
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            arm_action_client("/arm_controller/follow_joint_trajectory", true);

    // attempt to connect to the server(s):
    ROS_INFO("waiting for arm-control server: ");
    bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  
    control_msgs::FollowJointTrajectoryGoal goal; //consistent goal message for UR action service

    ros::spinOnce();
    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd); //start from current pose

    bool got_good_traj = false;

    //open, parse and check the trajectory file
    if (0 == read_traj_file(argv[1], des_trajectory)) {
        ROS_INFO("read file OK");
        got_good_traj = true;
    } else {
        ROS_ERROR("could not read playfile");
        exit(1);
    }

    //now have current arm pose and desired trajectory; splice in a motion from current arm pose
    // to first point of desired trajectory;
    //get first pt of traj: q_firstpoint
    Eigen::VectorXd q_firstpoint;
    trajectory_msgs::JointTrajectory approach_trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;
    q_firstpoint.resize(VECTOR_DIM);
        trajectory_point0 = des_trajectory.points[0];
        for (int i = 0; i < VECTOR_DIM; i++) { //copy from traj point to Eigen-type vector
            q_firstpoint[i] = trajectory_point0.positions[i];
        }
        //add this pt to path from current pose:
        des_path.push_back(q_firstpoint);
        //use traj stuffer to find build trajectory from current pose to first point of recorded traj
        //stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory)
        stuff_trajectory(des_path, approach_trajectory);


    //command the approach trajectory and wait for conclusion   
    g_done_move = true;


    goal.trajectory = approach_trajectory;
    g_done_move = false; //reset status trigger, so can check when done
        arm_action_client.sendGoal(goal, &armDoneCb); // we could also name additional callback functions here, if desired
 
    while (!g_done_move) {
        ROS_INFO("waiting on arm server to approach start of traj");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    // now send the desired trajectory from file, if there are any points left to execute
    if (got_good_traj && (des_trajectory.points.size() > 1)) {
        goal.trajectory = des_trajectory;
        g_done_move = false; //reset status trigger, so can check when done
        arm_action_client.sendGoal(goal, &armDoneCb); // we could also name additional callback functions here, if desired
    }
 

    while (!g_done_move) {
        ROS_INFO("waiting on arm server to execute playfile");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    return 0;
}




