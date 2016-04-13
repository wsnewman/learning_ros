// wsn pgm to receive Baxter trajectories and interpolate them smoothly
// as commands to Baxter;
// left arm only
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <actionlib/server/simple_action_server.h>

#include<baxter_trajectory_streamer/trajAction.h>
using namespace std;


ros::Publisher joint_cmd_pub_left;

// this is the interesting func: compute new qvec
// update isegment and qvec according to traj_clock; 
//if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false   

bool update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Vectorq7x1 qvec_prev, int &isegment, Vectorq7x1 &qvec_new) {
    trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
    Vectorq7x1 qvec, qvec_to, delta_qvec, dqvec;
    int nsegs = trajectory.points.size() - 1;
    double t_subgoal;
    //cout<<"traj_clock = "<<traj_clock<<endl;
    if (isegment < nsegs) {
        trajectory_point_to = trajectory.points[isegment + 1];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
        //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    } else {
        cout << "reached end of last segment" << endl;
        trajectory_point_to = trajectory.points[nsegs];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
        for (int i = 0; i < 7; i++) {
            qvec_new[i] = trajectory_point_to.positions[i];
        }
        cout << "final time: " << t_subgoal << endl;
        return false;
    }

    //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    while ((t_subgoal < traj_clock)&&(isegment < nsegs)) {
        //cout<<"loop: iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
        isegment++;
        if (isegment > nsegs - 1) {
            //last point
            trajectory_point_to = trajectory.points[nsegs];
            for (int i = 0; i < 7; i++) {
                qvec_new[i] = trajectory_point_to.positions[i];
            }
            cout << "iseg>nsegs" << endl;
            return false;
        }

        trajectory_point_to = trajectory.points[isegment + 1];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
    }
    //cout<<"t_subgoal = "<<t_subgoal<<endl;
    //here if have a valid segment:
    for (int i = 0; i < 7; i++) {
        qvec_to[i] = trajectory_point_to.positions[i];
    }
    delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
    double delta_time = t_subgoal - traj_clock;
    if (delta_time < dt_traj) delta_time = dt_traj;
    dqvec = delta_qvec * dt_traj / delta_time;
    qvec_new = qvec_prev + dqvec;
    return true;
}

class trajActionServer {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in baxter_traj_streamer/action/traj.action
    // the type "trajAction" is auto-generated from our name "traj" and generic name "Action"
    actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction> as_;

    // here are some message types to communicate with our client(s)
    baxter_trajectory_streamer::trajGoal goal_; // goal message, received from client
    baxter_trajectory_streamer::trajResult result_; // put results here, to be sent back to the client when done w/ goal
    baxter_trajectory_streamer::trajFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_);
    baxter_core_msgs::JointCommand left_cmd;
    trajectory_msgs::JointTrajectory new_trajectory; // member var to receive new traj's;
    int g_count; //=0; //just for testing
    bool working_on_trajectory; // = false;
    void cmd_pose_left(Vectorq7x1 qvec);
public:
    trajActionServer(); //define the body of the constructor outside of class definition

    ~trajActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction>::GoalConstPtr& goal);
};


//constructor starts populating the left_cmd message with joint names.
//In this implementation, ALL joint names must be specified and ALL joint angle commands must be specified
// for every command message.  Further, a fixed order of joints is assumed: from base to tip.
// This actually makes the joint naming irrelevant, but the joint-command message will nonetheless be fully
// populated according to the message type: baxter_core_msgs::JointCommand
trajActionServer::trajActionServer() :
as_(nh_, "leftArmTrajActionServer", boost::bind(&trajActionServer::executeCB, this, _1), false)
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of leftArmTrajActionServer...");
    // do any other desired initializations here...specific to your implementation
    //initializations:
    left_cmd.mode = 1; // set the command modes to "position"

    // define the joint angles 0-6 to be left arm, from shoulder out to wrist;
    left_cmd.names.push_back("left_s0");
    left_cmd.names.push_back("left_s1");
    left_cmd.names.push_back("left_e0");
    left_cmd.names.push_back("left_e1");
    left_cmd.names.push_back("left_w0");
    left_cmd.names.push_back("left_w1");
    left_cmd.names.push_back("left_w2");
    // do push-backs to establish desired vector size with valid joint angles
    for (int i = 0; i < 7; i++) {
        left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
    }
    g_count = 0; //carry-over from original action-server example; don't know if this will be useful in the future
    working_on_trajectory = false;
    as_.start(); //start the server running
}

//helper function: needs to convert Eigen-type vector into a C++ "vector" (variable-length array)
// within a field of a baxter_core_msgs::JointCommand message
void trajActionServer::cmd_pose_left(Vectorq7x1 qvec) {
    //member var left_cmd_ already has joint names populated; just need to update the joint-angle commands
    for (int i = 0; i < 7; i++) {
        left_cmd.command[i] = qvec[i];
    }
    joint_cmd_pub_left.publish(left_cmd);
}

//this is where the bulk of the work is done, interpolating between potentially coarse joint-space poses
// using the specified arrival times
void trajActionServer::executeCB(const actionlib::SimpleActionServer<baxter_trajectory_streamer::trajAction>::GoalConstPtr& goal) {
    double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
    int isegment;
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;

    Vectorq7x1 qvec, qvec0, qvec_prev, qvec_new;

    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.return_val = g_count; // we'll use the member variable result_, defined in our class
    //result_.traj_id = goal->traj_id;
    //cout<<"received trajectory w/ "<<goal->trajectory.points.size()<<" points"<<endl;
    // copy trajectory to global var:
    new_trajectory = goal->trajectory; // 
    // insist that a traj have at least 2 pts
    int npts = new_trajectory.points.size();
    if (npts  < 2) {
        ROS_WARN("too few points; aborting goal");
        as_.setAborted(result_);
    } else { //OK...have a valid trajectory goal; execute it
        //got_new_goal = true;
        //got_new_trajectory = true;
        ROS_INFO("Cb received traj w/ npts = %d",npts);
        //cout << "Cb received traj w/ npts = " << new_trajectory.points.size() << endl;
        //debug output...
        /*
        cout << "subgoals: " << endl;
        for (int i = 0; i < npts; i++) {
            for (int j = 0; j < 7; j++) { //copy from traj point to 7x1 vector
                cout << new_trajectory.points[i].positions[j] << ", ";
            }
            cout << endl;
        }
        */
        as_.isActive();

        working_on_trajectory = true;

        traj_clock = 0.0; // initialize clock for trajectory;
        isegment = 0; //initialize the segment count
        trajectory_point0 = new_trajectory.points[0]; //start trajectory from first point...should be at least close to
          //current state of system; SHOULD CHECK THIS
        for (int i = 0; i < 7; i++) { //copy from traj point to 7x1 Eigen-type vector
            qvec0[i] = trajectory_point0.positions[i];
        }
        cmd_pose_left(qvec0); //populate and send out first command  
        qvec_prev = qvec0;
        cout << "start pt: " << qvec0.transpose() << endl;
    }
    while (working_on_trajectory) {
        traj_clock += dt_traj;
        // update isegment and qvec according to traj_clock; 
        //if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false          
        working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);
        cmd_pose_left(qvec_new); // use qvec to populate object and send it to robot
        qvec_prev = qvec_new;
        //cout << "traj_clock: " << traj_clock << "; vec:" << qvec_new.transpose() << endl;
        ros::spinOnce();
        ros::Duration(dt_traj).sleep(); //update the outputs at time-step resolution specified by dt_traj
    }
    ROS_INFO("completed execution of a trajectory" );
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "left_arm_traj_interpolator_action_server"); // name this node     
    ros::NodeHandle nh;

    //this is how to command joint angles to Baxter;
    //publisher is global...would be better to make it a member of class
    joint_cmd_pub_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
   
    ROS_INFO("instantiating the trajectory interpolator action server: ");
    trajActionServer as; // create an instance of the class "trajActionServer"  

    ROS_INFO("left-arm interpolator ready to receive/execute trajectories");
    //main loop:
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(dt_traj).sleep();
    }
}
