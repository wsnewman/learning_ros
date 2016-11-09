// wsn pgm to receive trajectories for the arm7dof and interpolate them smoothly
// as commands to arm7dof;
// action server is called: trajActionServer

#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float64MultiArray.h>
#include<arm7dof_traj_as/trajAction.h>
using namespace std;


class TrajActionServer {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in arm7dof_traj_as/action/traj.action
    // the type "trajAction" is auto-generated from our name "traj" and generic name "Action"
    actionlib::SimpleActionServer<arm7dof_traj_as::trajAction> as_;

    // here are some message types to communicate with our client(s)
    arm7dof_traj_as::trajGoal goal_; // goal message, received from client
    arm7dof_traj_as::trajResult result_; // put results here, to be sent back to the client when done w/ goal
    arm7dof_traj_as::trajFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_);
    //baxter_core_msgs::JointCommand right_cmd;
    trajectory_msgs::JointTrajectory new_trajectory; // member var to receive new traj's;
    int g_count; //=0; //just for testing
    bool working_on_trajectory; // = false;
    //void cmd_pose(Vectorq7x1 qvec);
    void command_joints(Eigen::VectorXd q_cmd); 
    //void command_joints_inner_vel_loop(Eigen::VectorXd q_cmd);
    ros::Publisher j0_pub_, j1_pub_, j2_pub_, j3_pub_, j4_pub_, j5_pub_, j6_pub_;
    ros::Publisher vel_loop_jnt_cmd_publisher_;
    std_msgs::Float64MultiArray qdes_msg_;
    void initializePublishers();
public:

    TrajActionServer(ros::NodeHandle &nh);
    ~TrajActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<arm7dof_traj_as::trajAction>::GoalConstPtr& goal);
    bool update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
        int &isegment, Eigen::VectorXd &qvec_new);
      
};


//constructor starts populating the right_cmd message with joint names.
// However, a fixed order of joints is assumed: from base to tip.
// This actually makes the joint naming irrelevant, but the joint-command message 
//will nonetheless be fully populated 
TrajActionServer::TrajActionServer(ros::NodeHandle &nh) :nh_(nh),
as_(nh, "trajActionServer", boost::bind(&TrajActionServer::executeCB, this, _1), false)
// in the above initialization, we name the server "trajActionServer"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of TrajActionServer...");
    initializePublishers();
    // do push-backs to establish desired vector size with valid joint angles
    for (int i = 0; i < 7; i++) qdes_msg_.data.push_back(0.0);
    g_count = 0; //carry-over from original action-server example; don't know if this will be useful in the future
    working_on_trajectory = false;
    ROS_INFO("starting action server: trajActionServer ");
    as_.start(); //start the server running
}

void TrajActionServer::initializePublishers() {
        j0_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint0_position_controller/command", 1, true); 
        j1_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint1_position_controller/command", 1, true); 
        j2_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint2_position_controller/command", 1, true); 
        j3_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint3_position_controller/command", 1, true); 
        j4_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint4_position_controller/command", 1, true); 
        j5_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint5_position_controller/command", 1, true); 
        j6_pub_ =  nh_.advertise<std_msgs::Float64>("/arm7dof/joint6_position_controller/command", 1, true); 
        vel_loop_jnt_cmd_publisher_ =
            nh_.advertise<std_msgs::Float64MultiArray>("qdes_attractor_vec", 1, true);
        }
 
void TrajActionServer::command_joints(Eigen::VectorXd q_cmd) {
    std_msgs::Float64 qval_msg;
    
    qval_msg.data = q_cmd(0);
    j0_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(1);
    j1_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(2);
    j2_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(3);
    j3_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(4);
    j4_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(5);
    j5_pub_.publish(qval_msg);
    qval_msg.data = q_cmd(6);
    j6_pub_.publish(qval_msg);           
    for (int i=0;i<7;i++) qdes_msg_.data[i] = q_cmd(i);
    vel_loop_jnt_cmd_publisher_.publish(qdes_msg_);
}

// more general version--arbitrary number of joints
bool TrajActionServer::update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
        int &isegment, Eigen::VectorXd &qvec_new) {
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
    int njnts = qvec_prev.size();
    //cout<<"njnts for qvec_prev: "<<njnts<<endl;
    Eigen::VectorXd qvec, qvec_to, delta_qvec, dqvec;
    int nsegs = trajectory.points.size() - 1;
    //ROS_INFO("update_trajectory: nsegs = %d, isegment = %d",nsegs,isegment);
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
        
        for (int i = 0; i < njnts; i++) {
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
            //cout<<"next traj pt #jnts = "<<trajectory_point_to.positions.size()<<endl;
            for (int i = 0; i < njnts; i++) {
                qvec_new[i] = trajectory_point_to.positions[i];
            }
            //cout << "iseg>nsegs" << endl;
            return false;
        }

        trajectory_point_to = trajectory.points[isegment + 1];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
    }
    //cout<<"t_subgoal = "<<t_subgoal<<endl;
    //here if have a valid segment:
    //cout<<"njnts of trajectory_point_to: "<<trajectory_point_to.positions.size()<<endl;
    qvec_to.resize(njnts);
    for (int i = 0; i < njnts; i++) {
        qvec_to[i] = trajectory_point_to.positions[i];
    }
    delta_qvec.resize(njnts);
    delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
    double delta_time = t_subgoal - traj_clock;
    if (delta_time < dt_traj) delta_time = dt_traj;
    dqvec.resize(njnts);
    dqvec = delta_qvec * dt_traj / delta_time;
    qvec_new = qvec_prev + dqvec;
    return true;
}

//this is where the bulk of the work is done, interpolating between potentially coarse joint-space poses
// using the specified arrival times
void TrajActionServer::executeCB(const actionlib::SimpleActionServer<arm7dof_traj_as::trajAction>::GoalConstPtr& goal) {
    double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
    int isegment;
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;
    std_msgs::Float64 float64_msg;
    Eigen::VectorXd qvec, qvec0, qvec_prev, qvec_new;
    Vectorq7x1 q_vec7x1;
    Eigen::Affine3d affine_arm;
    Eigen::Vector3d Origin;
    // TEST TEST TEST
    //Eigen::VectorXd q_vec;
    //q_vec<<0.1,0.2,0.15,0.4,0.5,0.6,0.7;    

    ROS_INFO("in executeCB");

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.return_val = g_count; // we'll use the member variable result_, defined in our class
    //result_.traj_id = goal->traj_id;
    cout<<"received trajectory w/ "<<goal->trajectory.points.size()<<" points"<<endl;
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
        //trajectory_msgs::JointTrajectoryPoint trajectory_point0;
        //trajectory_point0 = new_trajectory.points[0];  
        //trajectory_point0 =  tj_msg.points[0];   
        //cout<<new_trajectory.points[0].positions.size()<<" =  new_trajectory.points[0].positions.size()"<<endl;
        //cout<<"size of positions[]: "<<trajectory_point0.positions.size()<<endl;
        cout << "subgoals: " << endl;
        int njnts; 
        for (int i = 0; i < npts; i++) {
            njnts = new_trajectory.points[i].positions.size();
            cout<<"njnts: "<<njnts<<endl;
            for (int j = 0; j < njnts; j++) { 
                cout << new_trajectory.points[i].positions[j] << ", ";
            }
            cout<<endl;
            cout<<"time from start: "<<new_trajectory.points[i].time_from_start.toSec()<<endl;
            cout << endl;
        }

        as_.isActive();

        working_on_trajectory = true;
        //got_new_trajectory=false;
        traj_clock = 0.0; // initialize clock for trajectory;
        isegment = 0;
        trajectory_point0 = new_trajectory.points[0];
        njnts = new_trajectory.points[0].positions.size();
        int njnts_new;
        qvec_prev.resize(njnts);
        qvec_new.resize(njnts);
        ROS_INFO("populating qvec_prev: ");
        for (int i = 0; i < njnts; i++) { //copy from traj point to Eigen type vector
            qvec_prev[i] = trajectory_point0.positions[i];
        }
        //cmd_pose_right(qvec0); //populate and send out first command  
        //qvec_prev = qvec0;
        cout << "start pt: " << qvec_prev.transpose() << endl;
    }
    while (working_on_trajectory) {
        traj_clock += dt_traj;
        // update isegment and qvec according to traj_clock; 
        //if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false 
        //ROS_INFO("traj_clock = %f; updating qvec_new",traj_clock);
        working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);
        //cmd_pose_right(qvec_new); // use qvec to populate object and send it to robot
        //ROS_INFO("publishing qvec_new as command");
        //davinciJointPublisher.pubJointStatesAll(qvec_new);
        command_joints(qvec_new);  //map these to all gazebo joints and publish as commands      
        qvec_prev = qvec_new;
       
        //cout << "traj_clock: " << traj_clock << "; vec:" << qvec_new.transpose() << endl;
        ros::spinOnce();
        ros::Duration(dt_traj).sleep();
    }
    ROS_INFO("completed execution of a trajectory" );
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm7dof_trajectory_action_server"); // name this node     
    ros::NodeHandle nh;
   
    ROS_INFO("instantiating the trajectory interpolator action server: ");
    TrajActionServer as(nh); // create an instance of the class "trajActionServer"  

    ROS_INFO("ready to receive/execute trajectories");
    //main loop:
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(dt_traj).sleep();
    }
}
