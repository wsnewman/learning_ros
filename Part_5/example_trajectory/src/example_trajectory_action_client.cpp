// example_trajectory_action_client: 
// see complementary server, "example_trajectory_action_server"
// this simple node populates a trajectory message and sends it to the trajectory action server for execution
// Run this together with minimal robot; start-up minimal robot with:   roslaunch minimal_robot_description minimal_robot.launch 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<example_trajectory/TrajActionAction.h>


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const example_trajectory::TrajActionResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "demo_trajectory_client_node"); // name this node 
        example_trajectory::TrajActionGoal goal; //instantiate a goal message compatible with our server, as defined in this package
        // we will command a limited-duration sinusoidal motion; define amplitude, frequency and duration
	double omega = 1.0; //rad/sec
        double amp = 0.5; //radians
	double start_angle= amp;
	double final_phase = 4*3.1415927; // radians--two periods
        
        //dt: break up trajectory into incremental commands this far apart in time
        // below, we will randomize this dt, just to illustrate that trajectories do not have to have fixed time steps
	double dt = 0.1; 
        
        actionlib::SimpleActionClient<example_trajectory::TrajActionAction> action_client("example_traj_action_server", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever
        ros::Duration sleep1s(1);
        if (!server_exists) {
            ROS_WARN("could not connect to server; retrying");
            bool server_exists = action_client.waitForServer(ros::Duration(1.0)); //
            sleep1s.sleep();
        }
        
        ROS_INFO("connected to action server");  // if here, then we connected to the server;


        // instantiate and populate a goal message:
	trajectory_msgs::JointTrajectory trajectory; //this contains an array (a vector) of trajectory points
	trajectory_msgs::JointTrajectoryPoint trajectory_point; //here is a single trajectory point, which will be populated and included in the trajectory
        
	//one specifies the text names of joints in the variable-length array (vector) "joint_names"
        // ROS allows for putting these in any order, and for specifying all or only a subset of joints
        // the current example is not so tolerant--it requires specifying ALL joint command values in a FIXED order
        // but for this simple example, there is only 1 joint anyway
	trajectory.joint_names.push_back("joint1");
	// repeat the above command for every joint of the robot, in some preferred order
	// joint position commands below must be specified in the same order as specified in the joint_names array
	int njnts = trajectory.joint_names.size(); // we specified this many joints;  need same size for position and velocity vectors
	trajectory_point.positions.resize(njnts);
	trajectory_point.velocities.resize(njnts);

	ROS_INFO("populating trajectory...");
	double final_time; //seconds
        double phase = 0.0; //radians        
	double time_from_start = 0.0; // seconds
	double q_des,qdot_des; //radians, radians/sec       
        
        //"phase" is a convenient variable = omega*time
	for (phase=0.0;phase<final_phase;phase+=omega*dt) {
		q_des = start_angle + amp*sin(phase); //here we make up a desired trajectory shape: q_des(t)
		qdot_des = amp*omega*cos(phase); // this is the time derivative of q_des; 
		trajectory_point.positions[0] = q_des; // do this for every joint, from 0 through njnts-1
		trajectory_point.velocities[0] = qdot_des; // and all velocities (in the server example, velocities will get ignored)
		time_from_start+= dt; //cumulative time from start of move
 		ROS_INFO("phase = %f, t = %f",phase,time_from_start);               
		//specify arrival time for this point--in ROS "duration" format
		trajectory_point.time_from_start = ros::Duration(time_from_start); //this converts from seconds to ros::Duration data type
		//append this trajectory point to the vector of points in trajectory:
		trajectory.points.push_back(trajectory_point);	
                //merely for illustration purposes, introduce a random time step; 
                // this shows that trajectory messages do not need a fixed time step
                // also, the dt values can be quite coarse in this example, for the purpose of illustrating the interpolation capability of the server
                dt = (rand() % 100 + 1)*0.01 ;     // rand() % 100 + 1 in the range 1 to 100, so dt is in the range from 0.01 to 1.0 sec
	}
	final_time = time_from_start; // the last assigned time; we should expect "success" back from our server after this long, else something went wrong
	int npts = trajectory.points.size();  //we just created this many points in our trajectory message
	ROS_INFO("populated trajectory with %d points",npts);
	//copy this trajectory into our action goal:	
	goal.trajectory = trajectory;

	//and send out the goal:
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired

        // wait for expected duration--plus some tolerance (chosen arbitrarily to be 2 seconds)
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // alternative: wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        }
        else {
            ROS_INFO("main: goal was reported as successfully executed.  Bye.");
        }
        

    return 0;
}

