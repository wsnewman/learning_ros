// reachability_from_above.cpp
// wsn, September 2016
// compute reachability, w/ z_tool_des pointing down;
// distance from gripper frame to flange frame is 0.1577
// cafe table is 0.79m above ground plane

// w/ Baxter on pedestal, torso is 0.93m above ground plane
// therefore, expect when fingers touch table, flange is at 0.79+0.1577 
// i.e., 0.0177m above torso
// search for reachability for flange over x_range = [0.4,1] , y_range= [-1,1] at z_range =[0,0.1]


#include <baxter_fk_ik/baxter_kinematics.h> 
#include <fstream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_reachability");
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    Baxter_fwd_solver baxter_fwd_solver;
    Baxter_IK_solver baxter_ik_solver;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    

    std::vector<Vectorq7x1> q_solns;
    Vectorq6x1 qsoln;

    Eigen::Affine3d a_tool_des; // expressed in DH frame  
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double x_des,y_des,z_des;
    double x_min = 0.4;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.0;
    double z_low = 0.0;
    double z_high = 0.1;
    double dx = 0.05;
    double dy = 0.05;
    Eigen::Vector3d p_des;
    int nsolns;
    std::vector<Eigen::Vector3d> reachable, approachable;
        
        for (double x_des = x_min;x_des<x_max;x_des+=dx) {
            for (double y_des = y_min; y_des<y_max; y_des+=dy) {
                p_des[0] = x_des;
                p_des[1] = y_des;
                p_des[2] = z_low; // test grasp pose; //z_high; //test approach pose
                a_tool_des.translation() = p_des;

                nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
                //valid = baxter_IK_solver_.improve_7dof_soln_wrt_torso(des_flange_affine, q_in, q_7dof_precise);
                if (nsolns>0) { //test grasp pose:
                        ROS_INFO("soln at x,y = %f, %f",p_des[0],p_des[1]);
                        reachable.push_back(p_des);
                }
                p_des[2] = z_high; // test grasp pose; //z_high; //test approach pose
                a_tool_des.translation() = p_des;

                nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
                //valid = baxter_IK_solver_.improve_7dof_soln_wrt_torso(des_flange_affine, q_in, q_7dof_precise);
                if (nsolns>0) { //test grasp pose:
                        ROS_INFO("soln at x,y = %f, %f",p_des[0],p_des[1]);
                        approachable.push_back(p_des);
                }                
                
            }
        }
    ROS_INFO("saving the results...");
    nsolns = reachable.size();
    ofstream outfile;
    outfile.open("reachable_x_y");
    
    for (int i=0;i<nsolns;i++) {
        p_des = reachable[i];
        outfile<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile.close();
    
    nsolns = approachable.size();
    ofstream outfile2;
    outfile2.open("approachable_x_y");
    
    for (int i=0;i<nsolns;i++) {
        p_des = approachable[i];
        outfile2<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile2.close();    
 
    return 0;
}
