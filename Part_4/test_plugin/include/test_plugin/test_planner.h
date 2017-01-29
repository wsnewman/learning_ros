#ifndef TEST_PLANNER_CPP
#define TEST_PLANNER_CPP

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>

namespace test_planner {
	class TestPlanner : public nav_core::BaseLocalPlanner {
	public:
		TestPlanner();
		//TestPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros);
		bool isGoalReached();
		bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
		bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
		void printPose(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2);
	private:
		ros::Time tg;
		unsigned int old_size;
		tf::TransformListener * handed_tf;
                std::vector< geometry_msgs::PoseStamped > g_plan_;
                ros::NodeHandle nh_;
                double controller_rate_, controller_dt_;
                int ipose_,nposes_;
	};	
};
#endif
