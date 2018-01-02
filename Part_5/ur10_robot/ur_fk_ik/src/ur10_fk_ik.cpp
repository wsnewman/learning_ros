// UR10 kinematics implementation file; start w/ fwd kin

#include <ur_fk_ik/ur_kin.h>

// function for use w/ both fwd and inv kin
// NOTE: q must be q in DH coords!!  use q_vec(i) + DH_q_offsets(i)

double sgn(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0; //should virtually never happen for floating-point numbers
}

Eigen::Matrix4d compute_A_of_DH(double a, double d, double alpha, double q) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q);
    double sq = sin(q);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    return A;
}

//alt fnc, just takes q in DH space and index of frame, 0 to 5, and uses global DH vars
Eigen::Matrix4d compute_A_of_DH(int i, double q_DH) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    double a = DH_a_params[i];
    double d = DH_d_params[i];
    double alpha = DH_alpha_params[i];
    //double q = q_ur + DH_q_offsets[i]; //convert to DH angles

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q_DH);
    double sq = sin(q_DH);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    return A;
}

//convert from DH angles to UR angles:
void UR10FwdSolver::q_DH_to_q_UR(Eigen::VectorXd q_soln_DH, Eigen::VectorXd &q_soln_UR){
    for (int i=0;i<6;i++) {
        q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i];
    }
    fit_joints_to_range(q_soln_UR);
}

//convert from UR angles to DH angles:
void UR10FwdSolver::q_UR_to_q_DH(Eigen::VectorXd q_soln_UR, Eigen::VectorXd &q_soln_DH){
    for (int i=0;i<6;i++)
        q_soln_DH[i] = q_soln_UR[i] + DH_q_offsets[i];
}

//assumes joints in UR coords; forces values to range -pi to pi
bool UR10FwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec) {
    bool fits = true;
    bool does_fit;
    double q;
    for (int i = 0; i < 6; i++) {
        q = qvec[i];
        does_fit = fit_q_to_range(q_lower_limits[i], q_upper_limits[i], q);
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}

bool UR10FwdSolver::fit_q_to_range(double q_min, double q_max, double &q) {
    while (q < q_min) {
        q += 2.0 * M_PI;
    }
    while (q > q_max) {
        q -= 2.0 * M_PI;
    }
    if (q < q_min)
        return false;
    else
        return true;
}

//solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid

bool solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns) {
    double r, cphi, sphi, phi, gamma;
    double KTOL = 0.000001; //arbitrary tolerance
    r = sqrt(A * A + B * B);
    phi = atan2(B, A);
    q_solns.clear();
    if (fabs(K) > fabs(r)) {
        ROS_WARN("K/r is too large for a cos/sin soln");
        return false; //if |K/r|>1, no solns
    }
    //could still have trouble w/ K=0...
    if (fabs(K) < KTOL) {
        ROS_WARN("K is too small for A,B,K trig soln: user error? ");
        return false; //illegal use of this fnc
    }
    gamma = acos(K / r);
    double soln1 = phi + gamma;
    double soln2 = phi - gamma;
    q_solns.push_back(soln1);
    q_solns.push_back(soln2);
    return true;
}

UR10FwdSolver::UR10FwdSolver() { //(const hand_s& hs, const atlas_frame& base_frame, double rot_ang) {

    ROS_INFO("fwd_solver constructor");
    // Transform from 6th link to end effector
    //  0, -1,  0,  0
    //  0,  0, -1,  0
    //  1,  0,  0,  0
    //  0,  0,  0,  1    
    A_tool = Eigen::MatrixXd::Zero(4, 4);
    A_tool(0, 1) = -1;
    A_tool(1, 2) = -1;
    A_tool(2, 0) = 1;
    A_tool(3, 3) = 1;
    Eigen::Matrix3d R_hand = Eigen:: MatrixXd::Identity(3,3);
    Eigen::Vector3d O_hand= Eigen::MatrixXd::Zero(3,1);
    //Eigen::Affine3d A_tool_wrt_flange_;
    A_tool_wrt_flange_.linear() = R_hand;
    A_tool_wrt_flange_.translation() = O_hand;
}

Eigen::MatrixXd UR10FwdSolver::jacobian(const Eigen::VectorXd& q_vec) {
    ROS_WARN("Jacobian function not implemented!");
    exit(1);
}
/*  IN CASE WANT JACOBIAN LATER...finish this
Eigen::MatrixXd irb120_hand_fwd_solver::get_Jacobian(const Vectorq6x1& q_vec) {
    solve(q_vec);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd J_ang = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd J_trans = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd zvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd rvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Matrix4d Apalm = A_mat_products[7];
    Eigen::MatrixXd O_palm = Apalm.block<3, 1>(0, 3);
    Eigen::Matrix4d Ai;
    Eigen::MatrixXd zvec, rvec;
    Eigen::Vector3d t1, t2;
    for (int i = 0; i < 6; i++) {
        Ai = A_mat_products[i];
        zvec = Ai.block<3, 1>(0, 2); //%strip off z axis of each movable frame
        zvecs.block<3, 1>(0, i) = zvec; //%and store them
        rvec = O_palm - Ai.block<3, 1>(0, 3); //%vector from origin of i'th frame to palm 
        rvecs.block<3, 1>(0, i) = rvec;
        J_ang.block<3, 1>(0, i) = zvecs.block<3, 1>(0, i);

        t1 = zvecs.block<3, 1>(0, i);
        t2 = rvecs.block<3, 1>(0, i);
        J_trans.block<3, 1>(0, i) = t1.cross(t2);
    }

    J.block<3, 6>(0, 0) = J_trans;
    J.block<3, 6>(3, 0) = J_ang;
    if (is_lhand(hs_))return mirror_J_to_lhand(J);
    return J;
}

 */

//provide qvec in UR coords; converts to DH coords before FK computation
Eigen::Affine3d UR10FwdSolver::fwd_kin_solve(const Eigen::VectorXd& q_vec_UR) {
    Eigen::Matrix4d M;
    Eigen::VectorXd q_vec_DH;
    q_vec_DH.resize(6);
    q_UR_to_q_DH(q_vec_UR,q_vec_DH);
    M = fwd_kin_solve_(q_vec_DH);
    Eigen::Affine3d A(M);
    return A;
}



Eigen::Matrix4d UR10FwdSolver::get_wrist_frame() {
    return A_mat_products[4];
}

//return soln out to tool flange; would still need to account for tool transform for gripper
// assumes jnt angles expressed in DH coords
Eigen::Matrix4d UR10FwdSolver::fwd_kin_solve_(const Eigen::VectorXd& q_vec_DH) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    for (int i = 0; i < NJNTS; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec_DH[i]);
        A_mats[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats[i] << std::endl;
    }

    A_mat_products[0] = A_mats[0];
    for (int i = 1; i < NJNTS; i++) {
        A_mat_products[i] = A_mat_products[i - 1] * A_mats[i];
    }

    //Eigen::Matrix4d A5;
    //A5 = A_mat_products[5]*A_tool; //apply a tool transform
    //A_mat_products[5] = A5;
    //std::cout<<" wrist position: "<<A5(0,3)<<", "<<A5(1,3)<<", "<<A5(2,3)<<std::endl;
    return A_mat_products[NJNTS - 1]; //flange frame
}


UR10IkSolver::UR10IkSolver() {
    //constructor: 
    L_humerus = DH_a_params[1];
    L_forearm = DH_a_params[2];
    ROS_INFO("UR10IKSolver constructor");
}

//solve IK; return solns, in UR coordinates, in q_ik_solns
int UR10IkSolver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  vector<Eigen::VectorXd> &q_ik_solns) {
    Eigen::VectorXd q_ik_soln;
    q_ik_soln.resize(NJNTS);
    q_ik_solns.clear();
    Eigen::Vector3d p_des = desired_hand_pose.translation();
    Eigen::Matrix3d R_des = desired_hand_pose.linear();
    Eigen::Vector3d b6_des = R_des.col(2); // direction of desired z-vector
    Eigen::Matrix4d T60; //target pose, expressed as a 4x4
    //convert affine to 4x4:
    T60 = Eigen::MatrixXd::Zero(4, 4);
    T60.block<3, 3>(0, 0) = R_des;
    T60.block<3, 1>(0, 3) = p_des;
    T60(3, 3) = 1.0;

    //w_des is origin O5; wrist is not spherical, but z5 and z6 intersect  
    double L6 = DH_d_params[5];
    Eigen::Vector3d w_des = p_des - L6*b6_des; // desired wrist position w/rt frame0
    //ROS_INFO("w_des: %f, %f, %f", w_des[0], w_des[1], w_des[2]);
    std::vector<double> q1_solns;
    std::vector<double> q5_solns_1a, q5_solns_1b;
    std::vector<double> q6_solns_1a, q6_solns_1b;

    q6dof_solns.clear();

    //solve for q1 first:
    if (!compute_q1_solns(w_des, q1_solns)) {
        return 0; // no solns
    }
    double q1a = q1_solns[0];
    double q1b = q1_solns[1];
    //compute q5 solutions
    //compute_q5_solns(p_des, q1_solns, q5_solns_1a, q5_solns_1b);

    //given q1 options, can compute target pose w/rt 1 frame(s)
    // transforms to express IK w/rt frames 1a, 1b
    Eigen::Matrix3d R10a, R10b, target_R61a, target_R61b;
    //target_R61a is Target orientation of frame 6 w/rt frame 1, using q1a soln

    Eigen::Matrix4d A1a, A1b; //4x4 transforms, frames 1 w/rt frame 0

    // T61a is from desired pose;
    //compute_A_of_DH(double a,double d,double q, double alpha); use q_vec(i) + DH_q_offsets(i)
    A1a = compute_A_of_DH(0, q1a); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1a+ DH_q_offsets[0] );
    A1b = compute_A_of_DH(0, q1b); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1b+ DH_q_offsets[0] );    
    Eigen::Matrix4d T61a, T61b; //target 4x4 wrt frame 1a, 1b
    Eigen::Vector3d w_wrt_1a, w_wrt_1b; //wrist point, expressed w/rt frames 1a and 1b
    Eigen::Vector3d b61a, b61b; // z-vector of desired flange frame (frame-6), for options q1a, q1b
    Eigen::Vector3d O1_wrt_0,O6_wrt_1; //origin of frame 1 w/rt frame 0
    R10a = A1a.block<3, 3>(0, 0);
    T61a = A1a.inverse() * T60;
    //cout<<"T60: "<<endl;
    //cout<<T60<<endl;
    //cout<<"A1a: "<<endl;
    //cout<<A1a<<endl;
    //cout<<"T61a: "<<endl;
    //cout<<T61a<<endl;
    target_R61a = T61a.block<3, 3>(0, 0);
    b61a = target_R61a.col(2);
    O6_wrt_1 = T61a.block<3,1>(0,3); // extract vector from origin of frame 1 to origin frame 6
    O1_wrt_0 = A1a.block<3,1>(0,3); //vector from origin frame 0 to origin frame 1
    // will be same for cases q1a, q1b
    R10b = A1b.block<3, 3>(0, 0);
    T61b = A1b.inverse() * T60;
    target_R61b = T61b.block<3, 3>(0, 0);
    b61b = target_R61b.col(2);
    
    //compute q5 based on orientation of T61a
    compute_q5_solns_from_R(target_R61a, q5_solns_1a);    
    //ROS_INFO("q5_DH solns from q1a via R61a: %f, %f",q5_solns_1a[0],q5_solns_1a[1]);
    //ROS_INFO("q5_UR solns from q1a via R61a: %f, %f",q5_solns_1a[0]-DH_q_offsets[4],
    //        q5_solns_1a[1]-DH_q_offsets[4]);
    
    compute_q5_solns_from_R(target_R61b, q5_solns_1b); //repeat for 2nd q1 option
    //compute q6 for cases q1a, q1b:
    //ROS_INFO("computing q6 for case q1a: ");
    compute_q6_solns(target_R61a, q5_solns_1a, q6_solns_1a);
    //ROS_INFO("computing q6 for case q1b: ");
    compute_q6_solns(target_R61b, q5_solns_1b, q6_solns_1b);  
    
    //compute T41, using q1, q5 and q6
    //there are 4 cases: (q1a, q5a1/q6a1), (q1a, q5a2/q6a2) 
    // and               (q1b, q5b1/q6b1), (q1b, q5b2/q6b2)
    // each of these has an elbow-up and elbow-down soln
    //compute T41a1:
    Eigen::Matrix4d T64a0,T64a1,T64b0,T64b1;
    T64a0= compute_A_of_DH(4, q5_solns_1a[0])*compute_A_of_DH(5, q6_solns_1a[0]);
    T64a1= compute_A_of_DH(4, q5_solns_1a[1])*compute_A_of_DH(5, q6_solns_1a[1]);
    //T64b0= compute_A_of_DH(4, q5_solns_1b[0])*compute_A_of_DH(5, q6_solns_1b[0]);
    //T64b1= compute_A_of_DH(4, q5_solns_1b[1])*compute_A_of_DH(5, q6_solns_1b[1]);
    Eigen::Matrix4d T41a0,T41a1;
    T41a0 = T61a*T64a0.inverse();
    T41a1 = T61a*T64a1.inverse();
    double O41xa,O41ya;
    O41xa = T41a0(0,3);
    O41ya = T41a0(1,3);
    //ROS_INFO("case T41a0: O4 w/rt frame 1: O4x, O4y = %f, %f ", O41xa,O41ya);
    double reachT41a0 = sqrt(O41xa*O41xa+O41ya*O41ya);
    vector<double> elbow_angs_a0, shoulder_angs_a0;
    //ROS_INFO("reach = %f",reachT41a0);
        //solve for elbow angles
    double L1 = DH_a_params[1];
    double L2 = DH_a_params[2];  
    
    //first 2 solns (elbow up/dn at q1a, first q5/q6 soln)
    if(solve_2R_planar_arm(O41xa,O41ya, L1, L2,shoulder_angs_a0,elbow_angs_a0) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_a0[0],elbow_angs_a0[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_a0[0],shoulder_angs_a0[1]);
        Eigen::Matrix4d T43a0,A32,A21;
        A32 = compute_A_of_DH(2, elbow_angs_a0[0]);
        A21 = compute_A_of_DH(1, shoulder_angs_a0[0]);
        T43a0 = A32.inverse()*A21.inverse()*T41a0;
        //cout<<"T43a0: "<<endl;
        //cout<<T43a0<<endl;
        double q_wrist = atan2(T43a0(1,0),T43a0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_DH,q_soln_UR;
        q_soln_DH.resize(6);
        q_soln_UR.resize(6);
        q_soln_DH[0]= q1a;
        q_soln_DH[1]= shoulder_angs_a0[0];
        q_soln_DH[2]= elbow_angs_a0[0];
        q_soln_DH[3] = q_wrist;
        q_soln_DH[4] = q5_solns_1a[0];
        q_soln_DH[5] = q6_solns_1a[0];
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //other elbow soln:
        A32 = compute_A_of_DH(2, elbow_angs_a0[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_a0[1]);
        T43a0 = A32.inverse()*A21.inverse()*T41a0;
        q_wrist = atan2(T43a0(1,0),T43a0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_a0[1];
        q_soln_DH[2]=elbow_angs_a0[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords       
    }
    //next 2 solns (3 and 4): (elbow up/dn at q1a, second q5/q6 soln)
    O41xa = T41a1(0,3);
    O41ya = T41a1(1,3);
    //ROS_INFO("case T41a1: O4 w/rt frame 1: O4x, O4y = %f, %f ", O41xa,O41ya);
    double reachT41a1 = sqrt(O41xa*O41xa+O41ya*O41ya);
    vector<double> elbow_angs_a1, shoulder_angs_a1;
    //ROS_INFO("reach = %f",reachT41a1);
    if(solve_2R_planar_arm(O41xa,O41ya, L1, L2,shoulder_angs_a1,elbow_angs_a1) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_a1[0],elbow_angs_a1[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_a1[0],shoulder_angs_a1[1]);
        Eigen::Matrix4d T43a1,A32,A21;
        A32 = compute_A_of_DH(2, elbow_angs_a1[0]);
        A21 = compute_A_of_DH(1, shoulder_angs_a1[0]);
        T43a1 = A32.inverse()*A21.inverse()*T41a1;
        //cout<<"T43a1: "<<endl;
        //cout<<T43a1<<endl;
        double q_wrist = atan2(T43a1(1,0),T43a1(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_DH,q_soln_UR;
        q_soln_DH.resize(6);
        q_soln_UR.resize(6);
        q_soln_DH[0]= q1a;
        q_soln_DH[1]= shoulder_angs_a1[0];
        q_soln_DH[2]= elbow_angs_a1[0];
        q_soln_DH[3] = q_wrist;
        q_soln_DH[4] = q5_solns_1a[1];
        q_soln_DH[5] = q6_solns_1a[1];
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //other elbow soln:
        A32 = compute_A_of_DH(2, elbow_angs_a1[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_a1[1]);
        T43a1 = A32.inverse()*A21.inverse()*T41a1;
        q_wrist = atan2(T43a1(1,0),T43a1(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_a1[1];
        q_soln_DH[2]=elbow_angs_a1[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords       
    }

    //repeat for alternative q1 soln, q1b;
    T64b0= compute_A_of_DH(4, q5_solns_1b[0])*compute_A_of_DH(5, q6_solns_1b[0]);
    T64b1= compute_A_of_DH(4, q5_solns_1b[1])*compute_A_of_DH(5, q6_solns_1b[1]);
    Eigen::Matrix4d T41b0,T41b1;
    T41b0 = T61b*T64b0.inverse();
    T41b1 = T61b*T64b1.inverse();
    double O41xb,O41yb;
    O41xb = T41b0(0,3);
    O41yb = T41b0(1,3);
    //ROS_INFO("case T41b0: O4 w/rt frame 1: O4x, O4y = %f, %f ", O41xb,O41yb);
    double reachT41b0 = sqrt(O41xb*O41xb+O41yb*O41yb);
    vector<double> elbow_angs_b0, shoulder_angs_b0;
    //ROS_INFO("reach = %f",reachT41b0);
   
    //at q1b, first 2 solns (elbow up/dn at q1b, first q5/q6 soln)
    if(solve_2R_planar_arm(O41xb,O41yb, L1, L2,shoulder_angs_b0,elbow_angs_b0) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_b0[0],elbow_angs_b0[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_b0[0],shoulder_angs_b0[1]);
        Eigen::Matrix4d T43b0,A32,A21;
        A32 = compute_A_of_DH(2, elbow_angs_b0[0]);
        A21 = compute_A_of_DH(1, shoulder_angs_b0[0]);
        T43b0 = A32.inverse()*A21.inverse()*T41b0;
        //cout<<"T43b0: "<<endl;
        //cout<<T43b0<<endl;
        double q_wrist = atan2(T43b0(1,0),T43b0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_DH,q_soln_UR;
        q_soln_DH.resize(6);
        q_soln_UR.resize(6);
        q_soln_DH[0]= q1b;
        q_soln_DH[1]= shoulder_angs_b0[0];
        q_soln_DH[2]= elbow_angs_b0[0];
        q_soln_DH[3] = q_wrist;
        q_soln_DH[4] = q5_solns_1b[0];
        q_soln_DH[5] = q6_solns_1b[0];
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //other elbow soln:
        A32 = compute_A_of_DH(2, elbow_angs_b0[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_b0[1]);
        T43b0 = A32.inverse()*A21.inverse()*T41b0;
        q_wrist = atan2(T43b0(1,0),T43b0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_b0[1];
        q_soln_DH[2]=elbow_angs_b0[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords       
    }
    //last 2 solns (elbow up/dn at q1b, second q5/q6 soln)    
    O41xb = T41b1(0,3);
    O41yb = T41b1(1,3);
    //ROS_INFO("case T41b1: O4 w/rt frame 1: O4x, O4y = %f, %f ", O41xb,O41yb);
    double reachT41b1 = sqrt(O41xb*O41xb+O41yb*O41yb);
    vector<double> elbow_angs_b1, shoulder_angs_b1;
    //ROS_INFO("reach = %f",reachT41b1);

    if(solve_2R_planar_arm(O41xb,O41yb, L1, L2,shoulder_angs_b1,elbow_angs_b1) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_b1[0],elbow_angs_b1[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_b1[0],shoulder_angs_b1[1]);
        Eigen::Matrix4d T43b1,A32,A21;
        A32 = compute_A_of_DH(2, elbow_angs_b1[0]);
        A21 = compute_A_of_DH(1, shoulder_angs_b1[0]);
        T43b1 = A32.inverse()*A21.inverse()*T41b1;
        //cout<<"T43b1: "<<endl;
        //cout<<T43b1<<endl;
        double q_wrist = atan2(T43b1(1,0),T43b1(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_DH,q_soln_UR;
        q_soln_DH.resize(6);
        q_soln_UR.resize(6);
        q_soln_DH[0]= q1b;
        q_soln_DH[1]= shoulder_angs_b1[0];
        q_soln_DH[2]= elbow_angs_b1[0];
        q_soln_DH[3] = q_wrist;
        q_soln_DH[4] = q5_solns_1b[1];
        q_soln_DH[5] = q6_solns_1b[1];
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //other elbow soln:
        A32 = compute_A_of_DH(2, elbow_angs_b1[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_b1[1]);
        T43b1 = A32.inverse()*A21.inverse()*T41b1;
        q_wrist = atan2(T43b1(1,0),T43b1(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_b1[1];
        q_soln_DH[2]=elbow_angs_b1[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords       
    }            
    return q_ik_solns.size(); 
}

//given desired tool-flange pose, find q1 solns:

bool UR10IkSolver::compute_q1_solns(Eigen::Vector3d w_des, std::vector<double> &q1_solns) {
    q1_solns.clear();
    //-s1*w_x + c1*w_y = -d4
    //solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid
    //solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
    if (!solve_K_eq_Acos_plus_Bsin(DH_d_params[3], w_des[1], -w_des[0], q1_solns)) {
        ROS_WARN("wrist point out of reach");
        return false; // no solns
    }
    double q1a = q1_solns[0]; //atan2(w_des(1), w_des(0));
    double q1b = q1_solns[1]; //q1a + M_PI; // given q1, q1+pi is also a soln
    //ROS_INFO("q1a, q1b = %f, %f", q1a, q1b);
    return true;
}

void UR10IkSolver::compute_q5_solns(Eigen::Vector3d p_des, std::vector<double> q1_solns,
        std::vector<double> &q5_solns_1a, std::vector<double> &q5_solns_1b) {
    //solve for q5:
    // w/rt frame 1, py = d4 + d5*c5
    // -s1*px + c1*py = d4 + d5*c5
    // q5 = +/- arccos[(-px*s1 +py*c1 -d4)/(d6)]
    double cq5;
    double q1a = q1_solns[0];
    double q1b = q1_solns[1];
    q5_solns_1a.clear(); //wrist q5 solns corresponding to first q1 soln
    q5_solns_1b.clear(); //and to second q1 soln

    cq5 = (-p_des[0] * sin(q1a) + p_des[1] * cos(q1a) - DH_d_params[3]) / DH_d_params[5];
    ROS_INFO("cq5a: %f", cq5);
    if (fabs(cq5) > 1.0) {
        ROS_WARN("no solns for q5a");
    } else {
        q5_solns_1a.push_back(acos(cq5)); // = acos(cq5);
        q5_solns_1a.push_back(-acos(cq5)); // = -q5a_solns[0];
        //ROS_INFO("q5a solns: %f, %f", q5_solns_1a[0], q5_solns_1a[1]);
    }

    //now for other q1 soln:
    cq5 = (-p_des[0] * sin(q1b) + p_des[1] * cos(q1b) - DH_d_params[3]) / DH_d_params[5];

    if (fabs(cq5) > 1.0) {
        ROS_WARN("no solns for q5b");
    } else {
        q5_solns_1b.push_back(acos(cq5));
        q5_solns_1b.push_back(-acos(cq5));
        //ROS_INFO("q5b solns: %f, %f", q5_solns_1b[0], q5_solns_1b[1]);
    }

}

//in this version, have that R61, third col = [c234*s5; s234*s5; -c5]
void UR10IkSolver::compute_q5_solns_from_R(Eigen::Matrix3d R61, std::vector<double> &q5_solns) {
    double c234s5 = R61(0,2);
    double s234s5 = R61(1,2);
    double c5 = -R61(2,2);
    double abs_s5 = sqrt(c234s5*c234s5+s234s5*s234s5);
    double q5a = atan2(abs_s5,c5);
    double q5b = atan2(-abs_s5,c5);
    q5_solns.clear();
    q5_solns.push_back(q5a);
    q5_solns.push_back(q5b);
}


//compute q6 for given desired orientation of 
bool UR10IkSolver::compute_q6_solns(Eigen::Matrix3d target_R61, std::vector<double> q5_solns,
        std::vector<double> &q6_solns) {
    double sign_s5, q6;
    q6_solns.clear();
    if (q5_solns.size()<1) {
        ROS_WARN("trying to compute q6, but q5_solns is empty");
        return false;
    }

   /* if multiply generic R_2/1(q2)*R_3/2(q3)*R_4/3(q4)*R_5/4(q5)*R_6/5(q6)
      last row of R_6/1(q2,q3,q4,q5,q6) is: [s5*c6; -s5*s6; c5]
    * use this to compute q6 w/ atan2
     */
    double a= target_R61(2,0); //c6 term
    double b = target_R61(2,1); //s6 term
    //compute q6 from first q5 soln:
    sign_s5 = sgn(sin(q5_solns[0]));     
    q6 = atan2(-sign_s5*b,sign_s5*a);
    //ROS_INFO("for first q5:  q6 (in DH coords) = %f",q6);
    //ROS_INFO("q6 in UR coords: %f",q6-DH_q_offsets[5]);
    q6_solns.push_back(q6);
    //repeat for 2nd q5 soln:
    sign_s5 = sgn(sin(q5_solns[1]));     
    q6 = atan2(-sign_s5*b,sign_s5*a);
    //ROS_INFO("for second q5: q6 (in DH coords) = %f",q6);
    q6_solns.push_back(q6);    
    return true;
}



bool UR10IkSolver::solve_2R_planar_arm_elbow_angs(double x_des, double y_des, double L1, double L2,
         vector<double> &q_elbow_solns) {
    const double fit_tol = 0.00001;
    double rsqrd = x_des*x_des +y_des*y_des;
    //ROS_INFO("planar arm: rsqrd, L1, L2 = %f, %f %f",rsqrd,L1,L2);
    //law of cosines: C^2 = A^2 + B^2 - 2ABcos(C_ang)
    double den = 2.0 * L1*L2;
    double num = rsqrd - L1*L1 - L2*L2;
    //test viability here...
    double c_ang = num / den;
    //ROS_INFO("c_ang, num, den = %f, %f, %f",c_ang,num,den);
    if (c_ang > 1.0) {
        //ROS_WARN("destination out of reach at full elbow extension");
        return false;
    }
    //cout<<"num, den, c4 = "<<num<<", "<<den<<", "<<c4<<endl;
    double s_ang = sqrt(1 - c_ang * c_ang);

    double q_elbow_a = atan2(s_ang, c_ang);
    //cout<<"q_elbow_a = "<<q_elbow_a<<endl;
    q_elbow_solns.clear();
    q_elbow_solns.push_back(q_elbow_a);
    
    double q_elbow_b = -q_elbow_a; //atan2(-s_ang, c_ang);
    q_elbow_solns.push_back(q_elbow_b);
    //cout<<"q_elbow_b = "<<q_elbow_b<<endl;
    return true;
}

//for planar arm in x-y plane, link lengths L1, L2, find 2 pairs of solns q1,q2
bool UR10IkSolver::solve_2R_planar_arm(double x_des, double y_des, double L1, double L2,
         vector<double> &q_shoulder_solns,vector<double> &q_elbow_solns) {
    const double fit_tol = 0.00001;
    double q_shoulder;
    //first, solve for elbow-angle solutions: expect 2 (up/dn):
    if (!solve_2R_planar_arm_elbow_angs( x_des,  y_des,  L1,  L2, q_elbow_solns)) {
        return false;
    }
    //given elbow angles, solve for corresponding shoulder angles:
    if(!solve_2R_planar_arm_shoulder_ang(x_des,y_des, L1, L2, q_elbow_solns[0], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);
    if (!solve_2R_planar_arm_shoulder_ang(x_des,y_des, L1, L2, q_elbow_solns[1], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);    
    return true;
}

//2R planar robot solution for shoulder angle
//given x_des, y_des, elbow_angle, solve for q_shoulder that points towards x_des,y_des;
bool UR10IkSolver::solve_2R_planar_arm_shoulder_ang(double x_des,double y_des, double L1, double L2,
           double q_elbow, double &q_shoulder) {
    //x = (L1+L2*c2)*c1 - (L2*s2)*s1
    double A = L1+L2*cos(q_elbow);
    double B = -L2*sin(q_elbow);
    const double R2_fit_err_tol = 0.000001;
    std::vector<double> q_solns;
    solve_K_eq_Acos_plus_Bsin(x_des,  A,  B, q_solns);
    //test which solution is correct:
    double x= L1*cos(q_solns[0]) + L2*cos(q_solns[0]+q_elbow);
    double y= L1*sin(q_solns[0]) + L2*sin(q_solns[0]+q_elbow);
    double fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
    //ROS_INFO("shoulder soln fit err 1: %f",fit_err);
    if (fit_err< R2_fit_err_tol) {
        q_shoulder = q_solns[0];
        return true; }
    q_shoulder = q_solns[1]; //try the alternative
    x= L1*cos(q_shoulder) + L2*cos(q_shoulder+q_elbow);
    y= L1*sin(q_shoulder) + L2*sin(q_shoulder+q_elbow);
    fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
    //ROS_INFO("shoulder soln fit err 2: %f",fit_err);
    if (fit_err< R2_fit_err_tol) {
        return true; }  
    //ROS_WARN("Error: 2R shoulder angle has no soln!");
    return false;
}

 


