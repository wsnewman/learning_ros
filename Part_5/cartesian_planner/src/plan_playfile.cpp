// plan_playfile: 
// wsn, April, 2016
//specify grasp x and y (presumed z=0)
// fixed approach direction, gripper_theta = M_PI/3.0;
// use cartesian path planner to compute viable horizontal move to approach grasp
// choose: yale-gripper z-axis is horizontal; x-axis points up
// try to move horizontally along gripper z axis, preserving orientation
// e.g. to grasp an upright cylinder (such as a cup, bottle, can...)
// save plan as an executable trajectory

// uses library of arm-motion planning functions
#include <cartesian_planner/cartesian_planner.h>
//#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

//some conversion utilities:
//getting a transform from a stamped transform is trickier than expected--there is not "get" fnc for transform

tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf) {
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}

//handy utility: convert from a tf::Transform object to an Eigen::Affine3d
//not used in this node

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

Eigen::Affine3d transformTFToEigen(const tf::StampedTransform &stf) {
    Eigen::Affine3d e;
    tf::Transform tf = get_tf_from_stamped_tf(stf); //strip out tf from stf
    e = transformTFToEigen(tf); //and use above fnc to convert to affine
    return e;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_playfile");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
    //some handy constants...
    Eigen::Matrix3d R_gripper_horiz;
    Eigen::Vector3d gripper_n_des, gripper_t_des, gripper_b_des;
    Eigen::Vector3d gripper_origin;
    bool found_path = false;

    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
    Eigen::Affine3d a_flange_start, a_hand_end, a_gripper_start;

    std::vector<Eigen::VectorXd> optimal_path;
    Eigen::VectorXd jspace_pose;
    //Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    ROS_INFO("instantiating a cartesian planner object: ");
    CartTrajPlanner cartTrajPlanner;
    tf::TransformListener tfListener; //create a transform listener
    tf::StampedTransform stf_gripper_wrt_flange;
    bool tferr = true;
    ROS_INFO("looking for transform yale gripper to right hand...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener.lookupTransform("right_hand", "yale_gripper_frame", ros::Time(0), stf_gripper_wrt_flange);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    Eigen::Affine3d A_gripper_wrt_flange;
    A_gripper_wrt_flange = transformTFToEigen(stf_gripper_wrt_flange);
    cout<<"A_gripper_wrt_flange: "<<endl;
    cout<<A_gripper_wrt_flange.linear()<<endl;
    cout<<"origin: "<<A_gripper_wrt_flange.translation().transpose()<<endl;



    ofstream outfile;
    outfile.open("computed_playfile.jsp");

    double gripper_theta = M_PI / 3.0; // choose approach heading
    double L_depart = 0.25; // distance to move along gripper z
    double x_des = 0.5;
    double y_des = 0.0;
    double z_des = 0.0;
    cout << "enter desired x_grasp (0.2 to 0.6): ";
    cin >> x_des;
    cout << "enter desired y_grasp (-0.6 to 0.3): ";
    cin >> y_des;
    gripper_n_des << 0, 0, 1; //gripper x-axis points straight up;
    gripper_b_des << cos(gripper_theta), sin(gripper_theta), 0;
    gripper_t_des = gripper_b_des.cross(gripper_n_des);
    R_gripper_horiz.col(0) = gripper_n_des;
    R_gripper_horiz.col(1) = gripper_t_des;
    R_gripper_horiz.col(2) = gripper_b_des;
    a_gripper_start.linear() = R_gripper_horiz;
    gripper_origin << x_des, y_des, z_des; //specify gripper pose at grasp position
    a_gripper_start.translation() = gripper_origin;

    //torso^A_flange = torso^A_gripper * gripper^A_flange
    a_flange_start = a_gripper_start * A_gripper_wrt_flange.inverse();

    cout<<"a_flange_start: "<<endl;
    cout<<a_flange_start.linear()<<endl;
    cout<<"origin: "<<a_flange_start.translation().transpose()<<endl;    

    a_hand_end.linear() = a_flange_start.linear();
    a_hand_end.translation() = a_flange_start.translation() - L_depart*gripper_b_des;


    //bool cartesian_path_planner(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path);
    found_path = cartTrajPlanner.cartesian_path_planner(a_flange_start, a_hand_end, optimal_path);

    if (found_path) {
        int nsamps = optimal_path.size();
        double arrival_time = 0.0;
        ROS_INFO("found path; x= %f, y= %f, gripper_theta = %f", x_des, y_des, gripper_theta);
        //outfile<<x_des<<", "<<y_des<<", "<<gripper_theta<<endl;
        for (int isamp = nsamps - 1; isamp >= 0; isamp--) { //save poses in reverse order
            jspace_pose = optimal_path[isamp];

            outfile << jspace_pose[0] << ", " << jspace_pose[1] << ", " << jspace_pose[2] << ", " << jspace_pose[3] << ", " << jspace_pose[4] << ", " << jspace_pose[05] << ", " << jspace_pose[6] << ", " << arrival_time << endl;
            arrival_time += 1.0;
        }

    } else {
        ROS_WARN("no path found; x= %f, y= %f, gripper_theta = %f", x_des, y_des, gripper_theta);
    }

    outfile.close();
    return 0;
}
