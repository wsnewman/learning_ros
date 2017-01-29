#include <test_plugin/test_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(test_planner::TestPlanner, nav_core::BaseLocalPlanner);

test_planner::TestPlanner::TestPlanner(){
	//I don't think we need anything here.
}

void test_planner::TestPlanner::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros){
	ros::NodeHandle nh(name);
	
	old_size = 0;
	handed_tf = tf;
}

bool test_planner::TestPlanner::isGoalReached(){
	//For demonstration purposes, sending a single navpoint will cause five seconds of activity before exiting.
	return ros::Time::now() > tg;
}

bool test_planner::TestPlanner::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan){
	//The "plan" that comes in here is a bunch of poses of varaible length, calculated by the global planner(?).
	//We're just ignoring it entirely, but an actual planner would probably take this opportunity
	//to store it somewhere and maybe update components that refrerence it.
	ROS_INFO("GOT A PLAN OF SIZE %lu", plan.size());
	//If we wait long enough, the global planner will ask us to follow the same plan again. This would reset the five-
	//second timer if I just had it refresh every time this function was called, so I check to see if the new plan is
	//"the same" as the old one first.
	if(plan.size() != old_size){
		old_size = plan.size();
		tg = ros::Time::now() + ros::Duration(5.0);
	}
	
	return true;
}

bool test_planner::TestPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){
	//This is the meat-and-potatoes of the plugin, where velocities are actually generated.
	//SPOILER ALERT: I just set a bunch of stuff to 1.
	cmd_vel.linear.x = 0.2;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
        cmd_vel.angular.z = 0.2;
	return true;
}
