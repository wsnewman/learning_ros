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
istream& operator >> ( istream& ins, record_t& record )
  {
  // make sure that the returned record contains only the stuff we read now
  record.clear();

  // read the entire line into a string (a CSV record is terminated by a newline)
  string line;
  getline( ins, line );

  // now we'll use a stringstream to separate the fields out of the line
  stringstream ss( line );
  string field;
  while (getline( ss, field, ',' ))
    {
    // for each field we wish to convert it to a double
    // (since we require that the CSV contains nothing but floating-point values)
    stringstream fs( field );
    double f = 0.0;  // (default value is 0.0)
    fs >> f;

    // add the newly-converted field to the end of the record
    record.push_back( f );
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
istream& operator >> ( istream& ins, data_t& data )
  {
  // make sure that the returned data only contains the CSV data we read here
  data.clear();

  // For every record we can read from the file, append it to our resulting data
  record_t record;
  while (ins >> record)
    {
    data.push_back( record );
    }

  // Again, return the argument stream as required for this kind of input stream overload.
  return ins;  
  }

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "playfile_jointspace"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    Eigen::VectorXd q_pre_pose_right,q_pre_pose_left;
    Eigen::VectorXd q_vec_right_arm,q_vec_left_arm;
    std::vector<Eigen::VectorXd> des_path_right, des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory_right,des_trajectory_left; // empty trajectories     
    
    //here are hard-coded joint angles for left and right arm poses
    cout<<"setting pre-poses: "<<endl;
    q_pre_pose_right.resize(7);
    q_pre_pose_left.resize(7);
       
        cout<<"instantiating a traj streamer"<<endl; // enter 1:";
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
 
        //get current pose of left and right arms:
    cout<<"right arm is at: "<<baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose()<<endl;
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd(); 
    cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

    q_vec_left_arm = baxter_traj_streamer.get_q_vec_left_arm_Xd(); 
    cout << "left-arm current state:" << q_vec_left_arm.transpose() << endl;
    
    des_path_right.push_back(q_vec_right_arm); //start from current pose
    des_path_left.push_back(q_vec_left_arm);

    
    if (argc!=3) {
      ROS_ERROR("must type two file names, right then left traj files, as command-line args");
    return 0;
    }
     //open the trajectory files:
     ifstream infile_right(argv[1]);
	if(!infile_right)			// file couldn't be opened
	{
		cerr << "Error: first file could not be opened; halting" << endl;
		exit(1);
	}    
     ifstream infile_left(argv[2]);
	if(!infile_left)			// file couldn't be opened
	{
		cerr << "Error: second file could not be opened; halting" << endl;
		exit(1);
	} 

    // Here is the data we want.
    data_t data_right, data_left;

  // Here is the file containing the data. Read it into data.
  infile_right >> data_right;
  infile_left >> data_left;
  // Complain if something went wrong.
  if (!infile_right.eof())
    {
    cout << "error reading file first file!\n";
    return 1;
    }
  if (!infile_left.eof())
    {
    cout << "error reading file second file!\n";
    return 1;
    }
   infile_left.close();
   infile_right.close();
  // Otherwise, list some basic information about the file.
  cout << "right CSV file contains " << data_right.size() << " records.\n";
  cout << "left CSV file contains " << data_left.size() << " records.\n";
  unsigned min_record_size = data_right[0].size();
  unsigned max_record_size = 0;
  for (unsigned n = 0; n < data_right.size(); n++) {
    if (max_record_size < data_right[ n ].size())
      max_record_size = data_right[ n ].size();
    if (min_record_size > data_right[ n ].size())
      min_record_size = data_right[ n ].size();    
  }
  if (max_record_size>8) {
      ROS_WARN("bad right-arm file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

  }
  if (min_record_size<8) {
            ROS_WARN("bad right-arm file");
    cout << "The smallest record has " << min_record_size << " fields.\n";
    return 1;

  }
  
  //check the left-arm file:
  min_record_size = data_left[0].size();
  max_record_size = 0;
  for (unsigned n = 0; n < data_left.size(); n++) {
    if (max_record_size < data_left[ n ].size())
      max_record_size = data_left[ n ].size();
    if (min_record_size > data_left[ n ].size())
      min_record_size = data_left[ n ].size();    
  }
  if (max_record_size>8) {
      ROS_WARN("bad left-arm file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

  }
  if (min_record_size<8) {
            ROS_WARN("bad left-arm file");
    cout << "The smallest record has " << min_record_size << " fields.\n";
    return 1;

  }  
  
  //data is valid; pack it up as trajectories and ship them out

    des_trajectory_right.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory_right.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
                                        // if using wsn's trajectory streamer action server
    des_trajectory_right.header.stamp = ros::Time::now();   
    //same for left-arm trajectory:
    des_trajectory_left.points.clear(); 
    des_trajectory_left.joint_names.clear(); 
    des_trajectory_left.header.stamp = ros::Time::now();       
  
    trajectory_msgs::JointTrajectoryPoint trajectory_point;//,trajectory_point2; 
    trajectory_point.positions.resize(7);
    double t_arrival;
    for (unsigned n = 0; n < data_right.size(); n++) {
       // pack trajectory points, one at a time:
       for (int i=0;i<7;i++) {
           trajectory_point.positions[i] = data_right[n][i];
       }
       t_arrival = data_right[n][7];
       trajectory_point.time_from_start = ros::Duration(t_arrival);
       des_trajectory_right.points.push_back(trajectory_point);
   }
     //repeat for left arm:
    for (unsigned n = 0; n < data_left.size(); n++) {
       // pack trajectory points, one at a time:
       for (int i=0;i<7;i++) {
           trajectory_point.positions[i] = data_left[n][i];
       }
       t_arrival = data_left[n][7];
       trajectory_point.time_from_start = ros::Duration(t_arrival);
       des_trajectory_left.points.push_back(trajectory_point);
   }    
    
   //now have the data in a trajectory; send it to the trajectory streamer action server to execute
   // goal objects compatible with the arm servers
    baxter_trajectory_streamer::trajGoal goal_right,goal_left;
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

    ROS_INFO("waiting for left-arm server: ");
    server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on left-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to left-arm action server"); // if here, then we connected to the server; 

    ROS_INFO("sending goals to left and right arms: ");
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); 
    left_arm_action_client.sendGoal(goal_left, &leftArmDoneCb); 
    
    return 0;
}



