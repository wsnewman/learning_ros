// example_trajectory_action_server: complementary server for trajectory messages
// accepts trajectory goals, interpolates between points linearly at specified dt
// The velocity commands are ignored by this simple interpolator

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<example_trajectory/TrajActionAction.h>
#include <std_msgs/Float64.h>
#include <vector>

using namespace std;

//put magic numbers/parameters here, making them more obvious
const double dt= 0.01; // decide what time resolution to use for interpolating the trajectory
const double min_dt = 0.0000001; // require that time steps have a minimum spacing
    
int g_count = 0;
bool g_count_failure = false;

//define a class TrajectoryActionServer, which will own an action server and a publisher
// ideally, it would also own a subscriber to joint_states, in order to assure starting poses are always safe (TODO)
//member method "send_joint_commands_()" must be customized to talk to a particular robot

class TrajectoryActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    ros::Publisher  jnt_cmd_publisher_; // need a publisher to talk to the joint controller
    // this class will own a "SimpleActionServer" called "as_".
    actionlib::SimpleActionServer<example_trajectory::TrajActionAction> as_;
    
    // here are some message types to communicate with our client(s)
    example_trajectory::TrajActionGoal goal_; // goal message, received from client
    example_trajectory::TrajActionResult result_; // put results here, to be sent back to the client when done w/ goal
    example_trajectory::TrajActionFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    
    void  send_joint_commands_(vector <double> q_cmd_jnts); // helper function to encapsulate details of how to talk to the controller; 

public:
    TrajectoryActionServer(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~TrajectoryActionServer(void) {
    }
    // Action Interface: this does all the work
    // uses goal message defined in "example_trajectory" package
    void executeCB(const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal);
};

//clumsy sytax for initializing objects within the constructor
// we will call the action server "example_traj_action_server"
TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle):nh_(*nodehandle),
   as_(nh_, "example_traj_action_server", boost::bind(&TrajectoryActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of TrajectoryActionServer...");
    // do any other desired initializations here...specific to your implementation
    ROS_INFO("Initializing Publisher");
    //next line specialized for use with minimal_joint_controller:
    jnt_cmd_publisher_ = nh_.advertise<std_msgs::Float64>("pos_cmd", 1, true); 
    as_.start(); //start the server running
}

// helper function to encapsulate details of how to talk to the controller; need to specialize this for your controller
void  TrajectoryActionServer::send_joint_commands_(vector <double> q_cmd_jnts) { 
    //for minimal_joint_controller, we only have one joint, and we command it with a publication
    std_msgs::Float64 q_cmd_msg; //need this to send commands to minimal_joint_controller
    q_cmd_msg.data = q_cmd_jnts[0]; // boring...only one component; really should check size, to make sure at least this component exists
    jnt_cmd_publisher_.publish(q_cmd_msg); // this is how we talk to the minimal_joint_controller; change this for your target controller
    ROS_INFO("commanding: %f",q_cmd_jnts[0]);
}
   
   
//here is where we do the work to act on goal requests
void TrajectoryActionServer::executeCB(const actionlib::SimpleActionServer<example_trajectory::TrajActionAction>::GoalConstPtr& goal) {
    // the class owns the action server, so we can use its member methods here
    // the goal is a pointer, and the action message contains a "trajectory" field; 
    trajectory_msgs::JointTrajectory trajectory = goal->trajectory; // goal-> notation is annoying, so copy the important stuff to a shorter named var

    ros::Rate rate_timer(1/dt); //here is a timer, consistent with our chosen time step

    int npts = trajectory.points.size(); // how many poses are contained in the goal request?
    ROS_INFO("received trajectory with %d points",npts); //debug/interpretation output
 
    int njoints = trajectory.joint_names.size(); // determine how many joints we have...though we really already know, for this example robot
    vector <double> q_prev_jnts; // for extension to multiple joints, create a variable-length array to hold joint commands
    vector <double> q_next_jnts; // we will interpolate between coarse joint commands, from q_prev_jnts to q_next_jnts
    vector <double> q_cmd_jnts;  // will contain interpolated joint command values
    q_prev_jnts.resize(njoints); // these arrays need to be this large to be consistent with dimension of input
    q_next_jnts.resize(njoints);
    q_cmd_jnts.resize(njoints);
    
    ROS_INFO("trajectory message commands %d joint(s)",njoints);
    double t_final = trajectory.points[npts-1].time_from_start.toSec(); // what is the last value of time_from_start specified?  convert to seconds
    ROS_INFO("t_final = %f",t_final);
    double t_previous = 0.0; 
    double t_next = trajectory.points[1].time_from_start.toSec();
    double t_stream=0.0; //start the clock from time 0
    double fraction_of_range=0.0; // this will be a ratio describing fraction of current segment completed
    double q_prev, q_next, q_cmd; 
    //start streaming points...interpolate, as needed.
    int ipt_next = 1;
    double t_range = t_next-t_previous;
    if (t_range< min_dt) {
        ROS_WARN("time step invalid in trajectory!");
        as_.setAborted(result_);
    }
    //put the starting joint commands here.  Really, these NEED to be the robot's actual joint commands!
    // this should be tested (and abort goal if not within some tolerance)
    q_prev_jnts = trajectory.points[ipt_next-1].positions;
    q_next_jnts = trajectory.points[ipt_next].positions;
    while (t_stream<t_final) {
        //compute desired pose at current time
        // see if t has stepped across current range, t_previous to t_next
        while (t_stream>t_next) { //find the next time range in the sequence
            ipt_next++;
            t_previous = t_next; //shift this down--former "next" is new "previous" in t_previous< t < t_next
            t_next = trajectory.points[ipt_next].time_from_start.toSec();
            t_range = t_next-t_previous;            
            if (t_range< min_dt) {
                ROS_WARN("time step invalid in trajectory!");
                as_.setAborted(result_);
                break;
            }   
            q_prev_jnts = trajectory.points[ipt_next-1].positions;  //find bounds for joint commands at previous and next time boundaries
            q_next_jnts = trajectory.points[ipt_next].positions;
        }        

        // if here, have valid range t_previous < t_stream < t_next
        //COMMENT OUT THE NEXT LINE TO SEE COARSE TRAJECTORY WITHOUT INTERPOLATION       
        fraction_of_range = (t_stream-t_previous)/(t_next-t_previous);

        //interpolate on the joint commands...linearly;  COULD BE BETTER, e.g. with splines
        for (int ijnt = 0;ijnt<njoints;ijnt++) {
           q_cmd_jnts[ijnt] = q_prev_jnts[ijnt] + fraction_of_range*(q_next_jnts[ijnt]-q_prev_jnts[ijnt]); //here is a vector of new joint commands;
        }
         ROS_INFO("t_prev, t_stream, t_next, fraction = %f, %f, %f, %f",t_previous,t_stream,t_next,fraction_of_range);
         ROS_INFO("q_prev, q_cmd, q_next: %f, %f, %f",q_prev_jnts[0],q_cmd_jnts[0],q_next_jnts[0]);
        //command the joints:  this implementation will depend on the specific interface to the robot;
        send_joint_commands_(q_cmd_jnts); // use helper function to encapsulate details of how to talk to the controller; 
        
        rate_timer.sleep();
        t_stream+=dt;
    }
    // output the final point; WATCH OUT HERE...trajectory messages are allowed to list joint commands in any order, as specified in
    // joint_names vector.  Further, it is generally allowed to specify commands for only a subset of joints (as listed in the joint_names vector),
    // with the presumption that all joints not listed should maintain their most recently commanded values.
    // I am shamelessly ignoring all of this, assuming joint commands are always specified for ALL joints, and that these commands are
    // always in the same, fixed order
    // This is a limitation if we want to control subsystems independently--e.g. hands on a steering wheel, vs feet on pedals, each
    // following their own respective algorithms.  For a single robot arm, though, subsystem control is not useful.
    
    q_cmd_jnts = trajectory.points[npts-1].positions;
    send_joint_commands_(q_cmd_jnts);
    //should populate result with something useful...
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
}

//the main program instantiates a TrajectoryActionServer, then lets the callbacks do all the work
int main(int argc, char** argv) {
    ros::init(argc, argv, "example_trajectory_action_node"); // name this node 

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type TrajectoryActionServer");
    TrajectoryActionServer as_object(&nh);
    
    
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin(); ros::ok() makes this routine easy to kill with ctl-C
        ros::Duration(0.1).sleep();
    }

    return 0;
}

